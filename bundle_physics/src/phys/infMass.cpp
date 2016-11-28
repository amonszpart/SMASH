//
// Created by bontius on 10/01/16.
//

#include "tracking/phys/infMass.h"

#include "tracking/phys/energyTerms/CoRBoundsTerm.h"
#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"
#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/energyTerms/impulseTerm.h"
#include "tracking/phys/energyTerms/conservationTerm.h"
#include "tracking/phys/energyTerms/poseTerm.h"
#include "tracking/phys/energyTerms/restrictCollPointTerm.h"
#include "tracking/phys/energyTerms/impl/restrictCollPointTerm.hpp"

#include "tracking/phys/bundleWithPhysics.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/physOutput.h"
#include "tracking/phys/partUtil.h"
#include "tracking/phys/physUtils.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/consts.h"
#include "tracking/phys/ceresUtil.h"
#include "tracking/phys/vis/visPhys.h"

#include "tracking/common/groupedTracks.h"
#include "tracking/common/mapper.h"

#include "tracking/vis/visualizer.h"
#include "ceres/impl/functorInfo.hpp"
#include "ceres/ceres.h"
#include "ceres/ceresUtil.h"

namespace tracking {
namespace bundle_physics {

int infMass(
    BundleWithPhysicsResult &out,
    CuboidsT const& cuboids_arg,
    GroupedTracks2d const& tracks2d,
    LinRgbsT    const& /*rgbs*/,
    FrameIdsT const&frameIds,
    Mapper const&mapper,
    Weights const&weights,
    BundleWithPhysicsResult const*const initial,
    std::string const& showFlags
)
{
    using ceres::FunctorInfo;
    using ceres::CostFunction;
    using ceres::CeresVector3;
    using ceres::CeresVector4;
    using ceres::CeresMatrix3;
    using ceres::MapCeresVector3;
    using ceres::MapConstCeresVector3;
    using ceres::MapCeresVector4;
    using ceres::MapConstCeresVector4;
    using ceres::CeresScalar;
    using Soup::vis::Visualizer;

    //typedef Cuboid::Scalar       Scalar;
    //typedef Cuboid::Vector3      Vector3;
    //typedef Cuboid::TranslationT TranslationT;
    //typedef Cuboid::QuaternionT  QuaternionT;
    const Consts  consts(weights.fps, frameIds);
    if (frameIds.size() > 3) {
        std::cerr << "expecting 3 frameIds: start, collision,end" << std::endl;
    }
    if (weights.initFlags && !initial) {
        std::cerr << "need initial not to be nullptr for initFlags>0" << std::endl;
        throw new Weights_InitDiscrepancyException("");
    }
    if (cuboids_arg.size() > 2) {
        std::cerr << "Unprepared for more, than two cuboids. First cuboid in indexer will have the finite mass" << std::endl;
        throw new std::runtime_error("");
    }

    //const Scalar                sceneScale(1.);
    const std::vector<CuboidId> participants = {0, 1};

    // Init indexer
    PhysIndexer indexer(frameIds, cuboids_arg.size());
    for (auto const &track2d : tracks2d)
        indexer.addPoint(track2d.getLabel());
    indexer.allocate();

    //ceres::LossFunction        *loss(new ceres::HuberLoss(100.));
    ceres::LossFunctionWrapper *poseLoss(
        new ceres::LossFunctionWrapper(
            new ceres::ScaledLoss(NULL, 1. /*weights.observedPoseWeight * weights.observedPoseWeight*/, ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP));

    ceres::Problem       problem;
    ceres::FunctorInfosT costFunctors;

    std::map<int, // cuboidId
        std::map<CollId, // collisionId
            std::pair<const Cuboid::StatesT::value_type *, // frameIdAndPoseLoc before
                const Cuboid::StatesT::value_type *  // frameIdAndPoseLoc after
            > > > closestPoses;

    CuboidsT finite, infinite;
    for (Cuboid const &cuboid : cuboids_arg) {
        if (cuboid.isMassFinite()) {
            finite.push_back(cuboid);
            if (infinite.size())
                throw new BundleWithPhysics_InfMass_FiniteCuboidsFirstAssumedException("");
        } else {
            infinite.push_back(cuboid);
        }
    } //...sort cuboids


    if (weights.observedPoseWeight > 0.)
        addPoseTerms(problem, indexer, costFunctors, frameIds, weights, finite, consts, poseLoss);
    if (weights.observedPosWeight > 0.)
        addParabolaTerms(problem, indexer, costFunctors, frameIds, weights, mapper, finite, consts, initial,
            /* use2dTerm: */ true);
    // init infinite positions
    for (Cuboid const& cuboid : infinite) {
        if (cuboid.getStates().size() && cuboid.getStates().begin()->second.hasPos()) {
            ceres::MapCeresVector3{indexer.getParabolaTranslation(1, 0)} =
                cuboid.getStates().begin()->second.getPosition().cast<CeresScalar>();
        }
    }

    for (CuboidId cuboidId = 0; cuboidId != CuboidId(finite.size()); ++cuboidId) {
        Cuboid const& cuboid(finite.at(cuboidId));

        //// ============================= Pose ============================= ////
        int lowerDist(std::numeric_limits<int>::max()),
            upperDist(std::numeric_limits<int>::max());

        for (PartId partId = consts.firstPartId; partId <= consts.lastPartId; ++partId) {
            for (CollId collId = partId - 1; collId <= partId; ++collId) {
                if (collId < 0 || collId > consts.lastCollId) { // edge cases
                    continue;
                }

                const FrameId partStart = getPartStart(partId, frameIds);
                const FrameId partEnd   = getPartEnd(partId, frameIds);

                for (const Cuboid::StatesT::value_type &frameIdAndState : cuboid.getStates()) {
                    const FrameId &frameId = frameIdAndState.first;
                    //const PoseLoc &poseLoc = frameIdAndState.second;
                    if (!frameIdAndState.second.hasPose())
                        continue;

                    // only bind to observations in this part
                    if (frameId <= partStart || frameId > partEnd)
                        continue;

                    // init
                    if (collId < partId) {// after part
                        int timeDist = std::abs(static_cast<int>(frameId) - static_cast<int>(frameIds.at(collId + 1)));
                        if (timeDist < upperDist) {
                            upperDist = timeDist;
                            closestPoses[cuboidId][collId].second = &frameIdAndState;
                        }
                    } else { // before part
                        int timeDist = std::abs(static_cast<int>(frameIds.at(collId + 1)) - static_cast<int>(frameId));
                        if (timeDist < lowerDist) {
                            lowerDist = timeDist;
                            closestPoses[cuboidId][collId].first = &frameIdAndState;
                        }
                    }
                } //...for cudoid states
            } //...for collisions
        } //...for parts

        // init mass
        {
            bool fixed(false);
            CeresScalar *mass = indexer.getMass(cuboidId, &fixed);
            mass[0] = 1.; //cuboid.getMass();
            if (weights.solveFlags & Weights::SOLVE) { // mass has not been added to problem, so can't fix it
                if (fixed) {
                    problem.SetParameterBlockConstant(mass);
                } else {
                    problem.SetParameterLowerBound(mass, 0, .1);
                    problem.SetParameterUpperBound(mass, 0, 10.0);
                }
            }
        } //...init mass
    } //...cuboids

    // init poses and time
    for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
        const int   partIdBefore   = getPartIdBefore(collId);
        const int   partIdAfter    = getPartIdAfter(collId);
        // init collision time
        CeresScalar *collisionTime = indexer.getCollisionTime(collId);
        {
            if (weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_COLLTIMES) {
                if (!initial || collId >= static_cast<CollId>(initial->collTimes.size()))
                    throw new BundleWithPhysics_InitialCollisionTimeFixedButNotProvidedException("");
                collisionTime[0] = initial->collTimes.at(collId);
            } else
                collisionTime[0] = frameIds.at(collId + 1) + 0.5;

            if (weights.fixFlags & Weights::FIX_FLAGS::FIX_COLLTIMES) {
                std::cout << "fixing time " << collId << std::endl;
                problem.SetParameterBlockConstant(collisionTime);
            } else {
                if (weights.bounds.collTime.lower > 0. && weights.bounds.collTime.upper > 0.) {
                    std::cout << "[" << __func__ << "] " << "fixing collTime to bounds:"
                              << weights.bounds.collTime.lower << "..." << weights.bounds.collTime.upper
                              << std::endl;
                    problem.SetParameterLowerBound(collisionTime, 0, weights.bounds.collTime.lower);
                    problem.SetParameterUpperBound(collisionTime, 0, weights.bounds.collTime.upper);
                } else {
                    problem.SetParameterLowerBound(collisionTime, 0, frameIds.at(collId + 1));
                    problem.SetParameterUpperBound(collisionTime, 0, getNext(frameIds.at(collId + 1)));
                }
            }
            std::cout << "collisionTime inited to " << collisionTime[0] << ", and bounded:" << frameIds.at(collId + 1) << ".." << getNext(frameIds.at(collId + 1)) << std::endl;
        } //...init collision time

        // init poses
        for (CuboidId cuboidId = 0; cuboidId != static_cast<CuboidId>(finite.size()); ++cuboidId) {
            const Cuboid &cuboid = finite.at(cuboidId);
            // initialize collision pose
            CeresScalar *const poseAddress = indexer.getCollisionPose(cuboidId, collId);
            if (closestPoses[cuboidId].find(collId) == closestPoses[cuboidId].end()) {
                std::cerr << "unexpected" << std::endl;
                throw new BundleWithPhysics_InitPoses_NoClosestPosesException("");
            }
            const auto        &closest    = closestPoses[cuboidId][collId];
            const CeresScalar prevTime    = CeresScalar(closest.first->first);
            const CeresScalar nextTime    = CeresScalar(closest.second->first);
            const CeresScalar timeElapsed = nextTime - prevTime;
            if (collisionTime[0] < prevTime || collisionTime[0] > nextTime || timeElapsed < 0.) {
                std::cout << "prevTime:" << prevTime << ", coll: " << collisionTime[0] << ", nextTime:" << nextTime << ", timeElapsed: " << timeElapsed << std::endl;
                throw new BundleWithPhysics_InitPoses_CollTimeNotBetweenObservationsException("");
            }

            const QuaternionT &prevPose = closest.first->second.getPose();
            const QuaternionT &nextPose = closest.second->second.getPose();
            const QuaternionT initPose  = prevPose.slerp((collisionTime[0] - prevTime) / timeElapsed, nextPose);
            if (weights.initFlags & Weights::USE_INPUT_POSES) {
                if (!initial)
                    throw new BundleWithPhysics_InitPoses_NoInitProvidedException("");
                if (initial->collisions.size() > 1)
                    throw new BundleWithPhysics_InitPoses_ASingleCollisionAssumedException("");
                std::cout << "initing pose to " << MapConstCeresVector4(poseAddress).transpose() << std::endl;
                quaternionToCeres(initial->collisions.at(0).states.first.at(cuboidId).getPose(), poseAddress);
                std::cout << "inited pose to " << MapConstCeresVector4(poseAddress).transpose() << std::endl;
            } else
                quaternionToCeres(initPose, poseAddress);

            if (weights.fixFlags & Weights::FIX_POSE) {
                std::cout << "fixing pose for " << cuboidId << " and coll " << collId << std::endl;
                problem.SetParameterBlockConstant(poseAddress);
                std::cout << "fixed pose" << std::endl;
            }

            // initialize momentum
            if (initial && (weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_MOMENTUM)) {
                if (initial->hasMomentum(cuboidId, partIdBefore)) {
                    CeresScalar *const momentumBefore = indexer.getMomentum(cuboidId, partIdBefore);
                    (MapCeresVector3(momentumBefore)) = initial->momenta.at(cuboidId).at(partIdBefore).cast<CeresScalar>();
                    if (weights.fixFlags & Weights::FIX_FLAGS::FIX_MOMENTA)
                        problem.SetParameterBlockConstant(momentumBefore);
                }
                if (initial->hasMomentum(cuboidId, partIdAfter)) {
                    CeresScalar *const momentumAfter = indexer.getMomentum(cuboidId, partIdAfter);
                    (MapCeresVector3(momentumAfter)) = initial->momenta.at(cuboidId).at(partIdAfter).cast<CeresScalar>();
                    if (weights.fixFlags & Weights::FIX_FLAGS::FIX_MOMENTA)
                        problem.SetParameterBlockConstant(momentumAfter);
                }
                //throw new BundleWithPhysics_InitMomentum_UseInputMomentumNotImplementedException("");
            } else if (!cuboid.isMassFinite()) {
                (MapCeresVector3(indexer.getMomentum(cuboidId, partIdBefore))) = CeresVector3::Zero();
                (MapCeresVector3(indexer.getMomentum(cuboidId, partIdAfter))) = CeresVector3::Zero();
                if (cuboid.getStates().size()) {
                    auto inputPose = cuboid.getStates().begin()->second.getPose();
                    quaternionToCeres(inputPose, poseAddress);
                } else {
                    poseAddress[0] = 1.;
                    poseAddress[1] = 0.;
                    poseAddress[2] = 0.;
                    poseAddress[3] = 0.;
                }
            } else {
                CeresScalar *mass = indexer.getMass(cuboidId);
                std::cout << "mass of cuboid " << cuboidId << ": " << mass[0] << std::endl;
                CeresVector3 initialI;
                cuboid.getIFromMass(initialI, mass[0]);
                std::cout << "I of cuboid " << cuboidId << ": " << initialI.transpose() << std::endl;
                const CeresMatrix3 R = initPose.toRotationMatrix().cast<CeresScalar>();
                CeresMatrix3 currI = R * initialI.asDiagonal() * R.transpose();
                std::cout << "currI of cuboid " << cuboidId << ":\n" << currI << std::endl;

                const QuaternionT omega0 = getEmpOmega(prevPose, initPose, Scalar((collisionTime[0] - prevTime) * consts.k_dt));
                {
                    CeresScalar *const momentumAddress = indexer.getMomentum(cuboidId, partIdBefore);
                    (MapCeresVector3(momentumAddress)) = currI * omega0.coeffs().head<3>().cast<CeresScalar>();
                    std::cout << "inited to momentum " << (MapCeresVector3(momentumAddress)).transpose()
                    << ", from omega " << omega0.coeffs().transpose()
                    << std::endl;
                }

                const QuaternionT omega1 = getEmpOmega(initPose, nextPose, Scalar((nextTime - collisionTime[0]) * consts.k_dt));
                {
                    CeresScalar *const momentumAddress = indexer.getMomentum(cuboidId, partIdAfter);
                    (MapCeresVector3(momentumAddress)) = currI * omega1.coeffs().head<3>().cast<CeresScalar>();
                    std::cout << "inited to momentum " << (MapCeresVector3(momentumAddress)).transpose()
                    << ", from omega " << omega1.coeffs().transpose()
                    << std::endl;
                }
                QuaternionT prev(QuaternionT::Unit());
                QuaternionT q1(Eigen::AngleAxisf(M_PI_2, Eigen::Vector3f::UnitX()));
                QuaternionT q2(Eigen::AngleAxisf(-M_PI_2, -Eigen::Vector3f::UnitX()));
                std::cout << "q1: " << q1.coeffs().transpose() << std::endl;
                std::cout << "q2: " << q2.coeffs().transpose() << std::endl;
                std::cout << "getEmp:" << getEmpOmega(prev, q1, 1.f).coeffs().transpose() << std::endl;
                std::cout << "getEmp:" << getEmpOmega(prev, q2, 1.f).coeffs().transpose() << std::endl;
            } //...init momentum
        } //...for cuboids
    } //...for collisions

    if (weights.conservationWeight > 0.)
        std::cerr << "ignoring conservation weight for infmass case...it's not conserved" << std::endl;
        //addConservationTerms(problem, indexer, costFunctors, frameIds, weights, finite.at(0), consts );
    if (weights.gravityDownWeight > 0. )
        addGravityTerm(problem, indexer, costFunctors, weights, consts);

    //// ============================= Run the solver ============================= ////
    ceres::Solver::Options options;
    {
        options.linear_solver_type           = ceres::SPARSE_NORMAL_CHOLESKY; // nonlin lsq
        //options.linear_solver_type           = ceres::CGNR; // nonlin lsq
        options.preconditioner_type          = ceres::JACOBI;
        //options.dynamic_sparsity = true;
        options.minimizer_progress_to_stdout = weights.doVis;
        options.max_num_iterations           = 4000;
        options.function_tolerance           = 1.e-12;
        options.parameter_tolerance          = 1.e-12;
        options.gradient_tolerance           = options.function_tolerance * 1.e-4;
        //std::cout << "linsearch: " << options.line_search_direction_type << std::endl;
        //options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
        //std::cout << "linsearch: " << options.line_search_direction_type << std::endl;
    }

    // solve
    {
        CollId const collId(0);
        // solve
        ceres::Solver::Summary summary;
        if (!(weights.solveFlags & Weights::NO_SOLVE)) {

            // =================== SOLVE 1 ==================
            ceres::Solve(options, &problem, &summary);
            if (weights.doVis) {
                std::cout << summary.FullReport() << "\n";
                printFunctors(costFunctors);
            }

            // gravity fixed
            problem.SetParameterBlockConstant(indexer.getParabolaRotationShared());

            CuboidId const objA = getParticipant(0, collId, participants);
            CuboidId const objB = getParticipant(1, collId, participants);

            problem.SetParameterBlockVariable(indexer.getParabolaTranslation(objA, collId));
            problem.SetParameterBlockVariable(indexer.getParabolaFreeParams(objA, getPartIdBefore(collId)));
            problem.SetParameterBlockVariable(indexer.getParabolaFreeParams(objA, getPartIdAfter(collId)));

            if (cuboids_arg.at(objB).isMassFinite()) {
                problem.SetParameterBlockVariable(indexer.getParabolaTranslation(objB, collId));
                problem.SetParameterBlockVariable(indexer.getParabolaFreeParams(objB, getPartIdBefore(collId)));
                problem.SetParameterBlockVariable(indexer.getParabolaFreeParams(objB, getPartIdAfter(collId)));
            }

            std::cout << "adding impulse terms" << std::endl;
            addImpulseTerms(problem, indexer, costFunctors, frameIds, weights, finite, participants, consts);
            addCorFunctorInfMass(problem, indexer, costFunctors, frameIds, weights, finite.at(0), consts);
            initCollPoint(problem, indexer, weights, participants, consts, initial);
            if (!cuboids_arg.at(objB).isMassFinite() && weights.collPointWeight > 0.)
                restrictCollPoint(cuboids_arg, collId, weights, problem, indexer, costFunctors);
            problem.SetParameterBlockConstant(indexer.getCollisionPoint(collId));

            // ================== SOLVE 2 ==================
            if (weights.solveStages & Weights::SOLVE_COUPLED) {
                std::cout << "solve2" << std::endl;
                ceres::Solve(options, &problem, &summary);
            }

            // ================== SOLVE 3 ==================
            problem.SetParameterBlockVariable(indexer.getCollisionPoint(collId));
            if (weights.solveStages & Weights::SOLVE_FREE_CP) {
                std::cout << "solve3" << std::endl;
                ceres::Solve(options, &problem, &summary);
            }

            // report
            if (weights.doVis) {
                std::cout << summary.FullReport() << "\n";
                printFunctors(costFunctors);
            }
        }

        // check for failure
        if (!(weights.solveFlags & bundle_physics::Weights::NO_SOLVE)) {
            if (summary.termination_type == ceres::FAILURE) {
                throw new BundleWithPhysics_CeresTerminationFailureException("Optimizer failure");
            }
        }
    } //...solve

    bundle_physics::output(out, cuboids_arg, tracks2d, frameIds, indexer, consts);

    // vis
    if (weights.doVis) {
        showPhysOutput( out, cuboids_arg, indexer, consts, showFlags);
    } //...vis

    return EXIT_SUCCESS;
} //...infMass

} //...ns bundle_physics
} //...ns tracking