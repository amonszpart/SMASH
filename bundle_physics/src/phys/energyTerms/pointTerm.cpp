//
// Created by bontius on 06/01/16.
//

#include "tracking/phys/initialize/assignments.h"
#include "tracking/phys/energyTerms/pointTerm.h"
#include "tracking/phys/energyTerms/impl/pointTerm.hpp"
#include "tracking/phys/energyTerms/compactnessTerm.h"
#include "tracking/phys/energyTerms/impl/compactnessTerm.hpp"
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

    void addPointTerms(
        ceres::Problem                   &       problem,
        PhysIndexer                      &       indexer,
        ceres::FunctorInfosT             &       costFunctors,
        GroupedTracks2d             const&       tracks2d,
        GroupsCuboidsT              const&       groupsCuboids,
        CuboidsT                    const&       cuboids,
        Mapper                      const&       mapper,
        FrameIdsT                   const&       frameIds,
        Weights                     const&       weights,
        Consts                      const&       consts,
        ceres::LossFunctionWrapper       * const pointLoss
    )
    {
        using ceres::CeresScalar;
        using ceres::CostFunction;
        char name[255];
        const CollId collId(0);

        //// ============================= Points ============================= ////
        for ( auto const& pair : tracks2d.getGroups() )
        {
            GroupId  const  groupId  = pair.first;
            CuboidId const  cuboidId = groupsCuboids.at( groupId );
            Cuboid   const& cuboid   = cuboids.at( cuboidId );
            for (TrackId const linTrackId : pair.second)
            {
                Track2D    const &      track2d      = tracks2d.getTrack(linTrackId);
                CeresScalar      *const pointAddress = indexer.getPointAddress(track2d.getLabel());

                for (auto const &idAndPoint : track2d.getPoints())
                {
                    const FrameId      frameId = idAndPoint.first;
                    const TrackPoint2D point   = idAndPoint.second;
                    const PartId       partId  = (frameId < frameIds.at(collId+1) ? getPartIdBefore(collId) : getPartIdAfter(collId) );


                    if (frameId > frameIds.back())
                        continue;

                    ceres::CostFunction *costFunction = PointCostFunctor::Create(
                        mapper.getIntrinsics(), point(0), point(1), frameId, consts.k_dt,
                        cuboid.getSize().data(), frameIds.at(collId+1), weights.poseIntegralSteps,
                        cuboid.getShape() );

                    CeresScalar *const sharedRot = indexer.getParabolaRotationShared();
                    CeresScalar *const sharedA = indexer.getParabolaSquaredParam();
                    CeresScalar *const translation = indexer.getParabolaTranslation(cuboidId, collId );
                    CeresScalar *const collPose = indexer.getCollisionPose( cuboidId, collId);
                    CeresScalar *const collTime = indexer.getCollisionTime(collId);
                    CeresScalar *const parabolaFree = indexer.getParabolaFreeParams(cuboidId, partId );
                    CeresScalar *const momentum = indexer.getMomentum(cuboidId, partId );
                    CeresScalar *const mass = indexer.getMass( cuboidId );
                    if (weights.solveFlags & Weights::SOLVE)
                    {
                        problem.AddResidualBlock(costFunction, pointLoss, pointAddress, sharedRot, sharedA, translation, parabolaFree, collPose, momentum, mass, collTime );
                    }

                    sprintf(name, "PointCostFunctor_%u_time%u", track2d.getLabel(), frameId);
                    costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {pointAddress, sharedRot, sharedA, translation, parabolaFree, collPose, momentum, mass, collTime}));
                } //...for frameIds

                // init 3D point location
                {
//                if (initial && initial->points3d->hasLabel(track2d.getLabel()))
//                {
//                    const TrackPoint3D &pnt3 = initial->points3d->getTrackByLabel(track2d.getLabel()).getPoint(FrameId(0));
//                    (MapCeresVector3(pointAddress)) = pnt3.getPoint();
//                    problem.SetParameterBlockConstant(pointAddress);
//                }
//                else
//                {
                    pointAddress[0] = rand() / static_cast<CeresScalar>(RAND_MAX) - CeresScalar(0.5); // * cuboid.getSize()(0);
                    pointAddress[1] = rand() / static_cast<CeresScalar>(RAND_MAX) - CeresScalar(0.5); // * cuboid.getSize()(1);
                    pointAddress[2] = rand() / static_cast<CeresScalar>(RAND_MAX) - CeresScalar(0.5); // * cuboid.getSize()(2);
//                }
                }
            } //...for tracks
        } //...for groups
    } //...addPointTerms()

    void addPointTerms(
        ceres::Problem                   &       problem,
        PhysIndexer                      &       indexer,
        PhysFunctorInfosT                &       costFunctors,
        TracksToCuboidsT            const&       assignments,
        GroupedTracks2d             const&       tracks2d,
        CuboidsT                    const&       cuboids,
        Mapper                      const&       mapper,
        FrameIdsT                   const&       frameIds,
        Weights                     const&       weights,
        Consts                      const&       consts
    )
    {
        //Eigen::Vector3f const kSize(1.,1.,1.); // dummy
        SHAPE           const kShape = SHAPE::BOX; // dummy
        using ceres::CeresScalar;
        using ceres::CostFunction;
        char name[255];
        const CollId collId(0);
        ceres::LossFunctionWrapper *compactnessLoss = new ceres::LossFunctionWrapper(new ceres::ScaledLoss(nullptr,1.,ceres::TAKE_OWNERSHIP),ceres::TAKE_OWNERSHIP);
        ceres::LossFunctionWrapper *pointLoss       = new ceres::LossFunctionWrapper(new ceres::ScaledLoss(nullptr,1.,ceres::TAKE_OWNERSHIP),ceres::TAKE_OWNERSHIP);
        ceres::LocalParameterization *quaternionParameterization = new ceres::QuaternionParameterization();
        std::set<CuboidId> cuboidIds;
        size_t cntPoints(0),cntCompactness(0);
        for (auto const& pair : tracks2d.getGroups()) {
            //GroupId  const  groupId  = pair.first;
            for (TrackId const linTrackId : pair.second)
            {
                Track2D    const&       track2d      = tracks2d.getTrack(linTrackId);
                TrackId    const&       trackId      = track2d.getTrackId();
                CeresScalar     *const  pointAddress = indexer.getPointAddress(trackId);
                CuboidId   const        cuboidId     = assignments.at(trackId);
                Cuboid     const&       cuboid       = cuboids.at( cuboidId );
                cuboidIds.insert(cuboidId);

                bool added = false;
                for (auto const &idAndPoint : track2d.getPoints()) {
                    const FrameId      frameId = idAndPoint.first;
                    const TrackPoint2D point   = idAndPoint.second;
                    const PartId       partId  = (frameId < frameIds.at(collId+1) ? getPartIdBefore(collId) : getPartIdAfter(collId) );

                    if (frameId > frameIds.back() || frameId < frameIds.front())
                        continue;

                    // points
                    {
                        ceres::CostFunction *costFunction = PointCostFunctor::Create(
                            mapper.getIntrinsics(), point(0), point(1), frameId, consts.k_dt, cuboid.getSize().data(), frameIds.at(collId + 1), weights.poseIntegralSteps, kShape);

                        CeresScalar *const sharedRot    = indexer.getParabolaRotationShared();
                        CeresScalar *const sharedA      = indexer.getParabolaSquaredParam();
                        CeresScalar *const translation  = indexer.getParabolaTranslation(cuboidId, collId);
                        CeresScalar *const collPose     = indexer.getCollisionPose(cuboidId, collId);
                        CeresScalar *const collTime     = indexer.getCollisionTime(collId);
                        CeresScalar *const parabolaFree = indexer.getParabolaFreeParams(cuboidId, partId);
                        CeresScalar *const momentum     = indexer.getMomentum(cuboidId, partId);
                        CeresScalar *const mass         = indexer.getMass(cuboidId);

                        ceres::ResidualBlockId blockId(nullptr);
                        if (weights.solveFlags & Weights::SOLVE) {
                            blockId = problem.AddResidualBlock(costFunction, pointLoss, pointAddress, sharedRot, sharedA, translation, parabolaFree, collPose, momentum, mass, collTime);
                            added   = true;
                            ++cntPoints;
                        }

                        sprintf(name, "PointCostFunctor_%u_time%u", track2d.getLabel(), frameId);
                        costFunctors.emplace_back(std::make_shared<TrackPhysFunctorInfo>(
                            name, costFunction, std::vector<CeresScalar *>{pointAddress, sharedRot, sharedA, translation, parabolaFree, collPose, momentum, mass, collTime}, trackId, blockId));
                    }
                } //...for frameIds

                // Compactness
                if (added && weights.compactnessWeight > 0.) {
                    ceres::CostFunction *costFunction = CompactnessCostFunctor::Create(cuboid.getSize().data());
                    ceres::ResidualBlockId blockId(nullptr);
                    if (weights.solveFlags & Weights::SOLVE) {
                        blockId = problem.AddResidualBlock(costFunction, compactnessLoss, pointAddress);
                        ++cntCompactness;
                    }
                    sprintf(name, "CompactnessCostFunctor_%u", track2d.getLabel());
                    costFunctors.emplace_back(std::make_shared<CompactnessPhysFunctorInfo>(
                        name, costFunction, std::vector<CeresScalar*>{pointAddress}, blockId));
                }

                // init 3D point location
                {
//                if (initial && initial->points3d->hasLabel(track2d.getLabel()))
//                {
//                    const TrackPoint3D &pnt3 = initial->points3d->getTrackByLabel(track2d.getLabel()).getPoint(FrameId(0));
//                    (MapCeresVector3(pointAddress)) = pnt3.getPoint();
//                    problem.SetParameterBlockConstant(pointAddress);
//                }
//                else
//                {
                    //auto const& pnt = mapper.to3D(track2d.getFirstPoint().getPoint(),randf());
                    pointAddress[0] = randf(); // pnt(0); //rand() / static_cast<CeresScalar>(RAND_MAX) - CeresScalar(0.5); // * cuboid.getSize()(0);
                    pointAddress[1] = randf(); // pnt(1); //rand() / static_cast<CeresScalar>(RAND_MAX) - CeresScalar(0.5); // * cuboid.getSize()(1);
                    pointAddress[2] = randf(); // pnt(2); // rand() / static_cast<CeresScalar>(RAND_MAX) - CeresScalar(0.5); // * cuboid.getSize()(2);
                    if (added) {
                        for (int dim = 0; dim != 3; ++dim) {
                            problem.SetParameterLowerBound(pointAddress, dim, -cuboid.getSize().maxCoeff());
                            problem.SetParameterUpperBound(pointAddress, dim, cuboid.getSize().maxCoeff());
                        }
                    }
//                }
                }
            } //...for tracks
        } //...for groups

        if (cntPoints) {
            double newPointWeight = weights.pointsUvWeight * weights.targetNormalization / static_cast<double>(cntPoints);
            pointLoss->Reset(chooseHuberOrTrivial(newPointWeight,weights.pointHuberParam),ceres::TAKE_OWNERSHIP);
//            if (weights.pointHuberParam > 0.) {
//                pointLoss->Reset(new ceres::ScaledLoss(new ceres::HuberLoss(weights.pointHuberParam),newPointWeight,ceres::TAKE_OWNERSHIP),ceres::TAKE_OWNERSHIP);
//            }
//            else
//                pointLoss->Reset(new ceres::ScaledLoss(nullptr                                      , newPointWeight,
//                    ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP);
            indexer.setWrapperInfo(POINT_TERM,{pointLoss,newPointWeight,cntPoints});
        }

        if (cntCompactness) {
            double newCompactnessWeight = weights.compactnessWeight * weights.targetNormalization / static_cast<double>(cntCompactness);
            compactnessLoss->Reset(chooseHuberOrTrivial(newCompactnessWeight,weights.compactnessHuberParam),ceres::TAKE_OWNERSHIP);
//            if (weights.compactnessHuberParam > 0.)
//                cnew ceres::ScaledLoss(new ceres::HuberLoss(weights.compactnessHuberParam), newCompactnessWeight,
//                    ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP);
//            else
//                compactnessLoss->Reset(new ceres::ScaledLoss(nullptr                                            , newCompactnessWeight,
//                    ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP);
            indexer.setWrapperInfo(COMPACTNESS_TERM,{compactnessLoss,newCompactnessWeight,cntCompactness});
        }

        for (CuboidId const& cuboidId : cuboidIds)
            problem.SetParameterization(indexer.getCollisionPose(cuboidId, collId),quaternionParameterization);
    } //...addPointTerms()
  } //...ns bundle_phsyics
} //...ns tracking