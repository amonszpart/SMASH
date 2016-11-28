//
// Created by bontius on 06/01/16.
//

#include "tracking/phys/energyTerms/impl/poseTerm.hpp"
#include "tracking/phys/energyTerms/poseTerm.h"
#include "tracking/phys/energyTerms/impl/minMomentumTerm.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/weights.h"
#include "tracking/phys/consts.h"
#include "tracking/phys/partUtil.h"
#include "tracking/annot/cuboid.h" // PoseLoc
#include "ceres/problem.h"

namespace tracking {
namespace bundle_physics {

void addPoseTerms(
    ceres::Problem              & problem,
    PhysIndexer                 & indexer,
    ceres::FunctorInfosT        & costFunctors,
    const FrameIdsT             & frameIds,
    const Weights               & weights,
    const CuboidsT              & cuboidsArg,
    //const std::vector<CuboidId> & participants,
    const Consts                & consts,
    ceres::LossFunctionWrapper  * const poseLoss
) {
    using ceres::CeresScalar;
    using ceres::CostFunction;
    using ceres::LossFunctionWrapper;
    using ceres::ScaledLoss;
    char name[255];

    ceres::LocalParameterization *quaternionParameterization = new ceres::QuaternionParameterization();
    size_t termCount(0);
    LossFunctionWrapper* const poseNormalizer {new LossFunctionWrapper(new ScaledLoss(poseLoss, 1., ceres::TAKE_OWNERSHIP),ceres::TAKE_OWNERSHIP)};

    CuboidsT _cuboids = cuboidsArg; // make local copy

//#warning Removed pose interpolation constraints! 4/5/2016
#if 0
    for (CuboidId cuboidId = 0; cuboidId != static_cast<CuboidId>(_cuboids.size()); ++cuboidId) {
        Cuboid &cuboid = _cuboids.at(cuboidId);
        auto it0 = cuboid.getStates().begin();
        while (!it0->second.hasPose() && it0 != cuboid.getStates().end())
            ++it0;
        if (it0 == cuboid.getStates().end() || std::next(it0) == cuboid.getStates().end())
            continue;

        for (auto it1 = std::next(it0); it1 != cuboid.getStates().end(); ++it1) {
            if (!it1->second.hasPose())
                continue;
            if (!it0->second.hasPose() || !it1->second.hasPose()) {
                std::cerr << "[" << __func__ << "] " << "assuming both cuboids having poses for interpolation..." << std::endl;
                throw new std::runtime_error("");
            }

            FrameId start = it0->first;
            QuaternionT const& pose0 = it0->second.getPose();
            FrameId stop = it1->first;
            QuaternionT const& pose1 = it1->second.getPose();
            for (FrameId frameId = getNext(start); frameId != stop; step(frameId)) {
                Scalar div = static_cast<Scalar>(stop)-static_cast<Scalar>(start);
                Scalar t = static_cast<Scalar>(frameId) - static_cast<Scalar>(start);
                Scalar dt = t / div;
                std::cout << "[" << __func__ << "] " << "slerping " << start << "_" << frameId << "_" << stop << " => " << dt << std::endl;
                QuaternionT ipol = pose0.slerp(dt, pose1);
                if (cuboid.hasPose(frameId)) {
                    std::cerr << "[" << __func__ << "] " << "there should be no pose yet" << std::endl;
                    throw new std::runtime_error("");
                }
                cuboid.setPose(frameId, ipol);
                if (!cuboid.hasPose(frameId)) {
                    std::cerr << "[" << __func__ << "] " << "there should be a pose by now..." << std::endl;
                    throw new std::runtime_error("");
                }
                std::cout << "[" << __func__ << "] " << "start:\n\t" << pose0.coeffs().transpose()
                          << "\n\t" <<  cuboid.getPose(frameId).coeffs().transpose()
                          << "\n\t" << pose1.coeffs().transpose() << std::endl;
            }

            // step
            it0 = it1;
        }
    } //...ipol for cuboids
#endif

    for (CuboidId cuboidId = 0; cuboidId != CuboidId(_cuboids.size()); ++cuboidId) {
        Cuboid const& cuboid(_cuboids.at(cuboidId));
        if (!cuboid.isMassFinite() )
            throw new BundleWithPhysics_PoseTerms_InfiniteMassException("");

        CeresScalar *const massAddress(indexer.getMass(cuboidId));

        // debug
        std::shared_ptr<QuaternionT> prevQ(nullptr);
        for (const Cuboid::StatesT::value_type &frameIdAndState : cuboid.getStates()) {
            if (!frameIdAndState.second.hasPose())
                continue;
            if (prevQ) {
                Scalar dot = prevQ->coeffs().dot(frameIdAndState.second.getPose().coeffs());
                if (dot < 0.)
                    std::cout << "[" << __func__ << "] " << "[cub" << cuboidId << "][frame" << frameIdAndState.first << "].dot: "
                              << dot << std::endl;
            } else {
//                std::cout << (frameIdAndState.second.hasPose() ? "HasPose" : "NoPose") << std::endl;
                prevQ = std::make_shared<QuaternionT>(frameIdAndState.second.getPose());
            }
            *prevQ = frameIdAndState.second.getPose();
        }

        for (PartId partId = consts.firstPartId; partId <= consts.lastPartId; ++partId) {
            CeresScalar *const momentumAddress = indexer.getMomentum(cuboidId, partId);

            for (CollId collId = partId - 1; collId <= partId; ++collId) {
                if (collId < 0 || collId > consts.lastCollId)
                    continue;

                CeresScalar *collTimeAddress = indexer.getCollisionTime(collId);
                CeresScalar *poseAddress     = indexer.getCollisionPose(cuboidId, collId);

                FrameId const partStart = getPartStart(partId, frameIds);
                FrameId const partEnd   = getPartEnd(partId, frameIds);
                for (const Cuboid::StatesT::value_type &frameIdAndState : cuboid.getStates()) {
                    const FrameId &frameId = frameIdAndState.first;
                    const PoseLoc &poseLoc = frameIdAndState.second;

                    // only bind to observations in this part
                    if (frameId < partStart || frameId > partEnd)
                        continue;

                    if (!poseLoc.hasPose())
                        continue;
                    CostFunction* costFunction = PoseIntegralCostFunctor::Create(
                        weights.observedPoseWeight, consts.k_dt, cuboid.getSize().data(),
                        poseLoc.getPose().coeffs(), frameId, frameIds.at(collId + 1),
                        weights.poseIntegralSteps, cuboid.getShape());
                    ++termCount;

                    if (weights.solveFlags & Weights::SOLVE) {
                        problem.AddResidualBlock(costFunction, poseLoss, poseAddress, momentumAddress, massAddress, collTimeAddress);
                        problem.SetParameterization(poseAddress, quaternionParameterization);
                    }
                    sprintf(name, "PoseIntegral_cub%d_time%u", cuboidId, static_cast<unsigned>(frameId));
                    costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {poseAddress, momentumAddress, massAddress, collTimeAddress}));
                } //...for cudoid states
            } //...for collisions

            if ( weights.minMomentumWeight > 0. ) {
                CostFunction *costFunction = MinMomentumFunctor::Create(1.);
                ceres::LossFunctionWrapper *momentumLoss(
                    new ceres::LossFunctionWrapper(
                        new ceres::ScaledLoss(NULL, weights.minMomentumWeight * weights.minMomentumWeight, ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP));
                if (weights.solveFlags & Weights::SOLVE)
                    problem.AddResidualBlock(costFunction, momentumLoss, momentumAddress);
                sprintf(name, "MinMomentum_cub%d_part%d", cuboidId, partId);
                costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {momentumAddress}));
            }
        } //...for parts
    } //...for cuboids
    poseNormalizer->Reset(new ceres::ScaledLoss(poseLoss,1./static_cast<float>(termCount),ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP);
} //...addPoseTerms()

} //...ns bundle_phsyics
} //...ns tracking