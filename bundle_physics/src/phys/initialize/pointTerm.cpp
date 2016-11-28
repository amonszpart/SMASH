//
// Created by bontius on 01/04/16.
//

#include "tracking/phys/energyTerms/compactnessTerm.h"
#include "tracking/phys/energyTerms/impl/batchPointTerm.hpp"
#include "tracking/phys/energyTerms/impl/pointTerm.hpp"
#include "tracking/phys/energyTerms/pointTerm.h"
#include "tracking/phys/initialize/assignments.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/weights.h"
#include "tracking/phys/consts.h"
#include "tracking/phys/partUtil.h"
#include "tracking/annot/cuboid.h"
#include "tracking/common/util/util.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/unique_ptr.h"
#include "tracking/phys/physFunctorInfo.h"
#include "ceres/problem.h"
#include <memory>

namespace tracking {
  namespace bundle_physics {

    void addBatchPointTerms(
        ceres::Problem         &problem,
        PhysIndexer            &indexer,
        PhysFunctorInfosT      &costFunctors,
        TracksToCuboidsT const &assignments,
        GroupedTracks2d  const &tracks2d,
        CuboidsT         const &cuboids,
        Mapper           const &mapper,
        FrameIdsT        const &frameIds,
        Weights          const &weights,
        Consts           const &consts,
        ceres::LossFunctionWrapper *const pointLoss
    ) {
        using ceres::CeresScalar;
        using ceres::CostFunction;

        SHAPE const kShape = SHAPE::BOX; // dummy
        CollId const collId(0);
        char         name[255];

        // Gather point addresses
        struct BatchHelper { TrackPoint2D const& pnt; CeresScalar *const pointAddress; TrackId const trackId; };
        std::map< CuboidId, std::map< PartId, std::map<FrameId,std::list<BatchHelper> > > > batches;
        for (auto const &pair : tracks2d.getGroups()) {
//            GroupId const groupId = pair.first;
            for (TrackId const linTrackId : pair.second) {
                Track2D const  &track2d = tracks2d.getTrack(linTrackId);
                TrackId const  &trackId = track2d.getTrackId();
                CuboidId const cuboidId = assignments.at(trackId);
                Cuboid const& cuboid = cuboids.at(cuboidId);
                CeresScalar *const pointAddress = indexer.getPointAddress(trackId);

                for (auto const &idAndPoint : track2d.getPoints()) {
                    const FrameId      frameId = idAndPoint.first;
                    const TrackPoint2D point   = idAndPoint.second;
                    const PartId       partId  = (frameId < frameIds.at(collId + 1) ? getPartIdBefore(collId) : getPartIdAfter(collId));

                    if (frameId < frameIds.front() || frameId > frameIds.back())
                        continue;

                    batches[cuboidId][partId][frameId].push_back({point,pointAddress,trackId});
                    (Eigen::Map<Eigen::Matrix<CeresScalar,3,1> >(pointAddress)) = mapper.to3D(track2d.getFirstPoint().getPoint(),randf()).cast<CeresScalar>();

                    // Compactness
                    if (0)
                    {
                        ceres::CostFunction *costFunction = CompactnessCostFunctor::Create(cuboid.getSize().data());
                        ceres::ResidualBlockId blockId(nullptr);
                        if (weights.solveFlags & Weights::SOLVE) {
                            blockId = problem.AddResidualBlock(costFunction, new ceres::ScaledLoss(nullptr,0.01,ceres::TAKE_OWNERSHIP), pointAddress);
                        }
                        sprintf(name, "CompactnessCostFunctor_%u", track2d.getLabel());
                        costFunctors.emplace_back(std::make_shared<PhysFunctorInfo>(
                            name, costFunction, std::vector<CeresScalar*>{pointAddress}, blockId));
                    }
                } //...for frames
            } //...for points
        } //...for groups

        // Setup
        CeresScalar *const sharedRot = indexer.getParabolaRotationShared();
        CeresScalar *const sharedA   = indexer.getParabolaSquaredParam();
        CeresScalar *const collTime  = indexer.getCollisionTime(collId);

        for (auto const& byCuboidId : batches ) {
            CuboidId const cuboidId = byCuboidId.first;
            Cuboid const &cuboid = cuboids.at(cuboidId);

            CeresScalar *const mass         = indexer.getMass(cuboidId);
            CeresScalar *const translation  = indexer.getParabolaTranslation(cuboidId, collId);
            CeresScalar *const collPose     = indexer.getCollisionPose(cuboidId, collId);

            for (auto const& byPartId : byCuboidId.second) {
                PartId       const&      partId       = byPartId.first;
                CeresScalar       *const parabolaFree = indexer.getParabolaFreeParams(cuboidId, partId);
                CeresScalar       *const momentum     = indexer.getMomentum(cuboidId, partId);
                for (auto const& byFrameId : byPartId.second) {
                    FrameId const& frameId = byFrameId.first;

                    std::map<CeresScalar*,TrackId> trackIds;
                    std::list<Eigen::Vector2d> observations;
                    std::vector<CeresScalar*> pointUnknowns;
                    std::for_each(byFrameId.second.begin(),byFrameId.second.end(),[&observations,&pointUnknowns,&trackIds](BatchHelper const& helper) {
                        observations.push_back({helper.pnt(0), helper.pnt(1)});
                        pointUnknowns.push_back(helper.pointAddress);
                        trackIds[helper.pointAddress] = helper.trackId;
                    });
                    std::vector<CeresScalar*> commonUnknowns = {
                        sharedRot,sharedA,translation,parabolaFree,collPose,momentum,mass,collTime};

                    std::vector<CeresScalar*   > unknowns = commonUnknowns;
                    std::list  <Eigen::Vector2d> currObservations;
                    std::vector<TrackId        > currTrackIds;
                    auto obsIt = observations.begin();
                    auto puIt  = pointUnknowns.begin();
                    while (obsIt != std::end(observations) && puIt != std::end(pointUnknowns)) {
                        currObservations.push_back(*obsIt);
                        unknowns        .push_back(*puIt );
                        currTrackIds    .push_back(trackIds.at(*puIt));
                        if (!(currObservations.size()%1) || std::next(obsIt) == std::end(observations)) {
                            ceres::CostFunction *costFunction = BatchPointCostFunctor::Create(
                                mapper.getIntrinsics(), frameId, consts.k_dt, cuboid.getSize().data(),frameIds.at(collId+1), weights.poseIntegralSteps, kShape, currObservations);
                            ceres::ResidualBlockId residualBlockId = problem.AddResidualBlock(costFunction,pointLoss,unknowns);
                            std::string trackIdsString("");
                            std::for_each(std::begin(currTrackIds),std::end(currTrackIds),[&trackIdsString](TrackId const& trackId){trackIdsString+=std::to_string(trackId)+"_";});
                            sprintf(name, "PointCostFunctor_%stime%u", trackIdsString.c_str(), frameId);
                            costFunctors.emplace_back(std::make_shared<TrackPhysFunctorInfo>(
                                name,costFunction,unknowns,currTrackIds,residualBlockId));
                            currObservations.clear();
                            currTrackIds    .clear();
                            unknowns = commonUnknowns;
                        }
                         ++obsIt; ++puIt;
                    } //...while blocks
                } //...byFrameId
            } //...byPartId
        } //...byCuboidId
    } //...addPointTerms()

  } //...ns bundle_physics
} //...ns tracking