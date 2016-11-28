#include "tracking/phys/energyTerms/poseTerm.h"
#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/energyTerms/impulseTerm.h"
#include "tracking/phys/energyTerms/conservationTerm.h"
#include "tracking/phys/energyTerms/impl/restrictCollPointTerm.hpp"
#include "tracking/phys/energyTerms/restrictCollPointTerm.h"
#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"
//#include "tracking/phys/energyTerms/pointTerms.h"
#include "tracking/phys/energyTerms/CoRBoundsTerm.h"
#include "tracking/phys/bundleWithPhysics.h"
#include "tracking/phys/physOutput.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/partUtil.h"
#include "tracking/phys/physUtils.h"
#include "tracking/phys/vis/vis.h"

#include "tracking/phys/ceresUtil.h"
//#include "tracking/annot/bundleAdjuster.h"

#include "tracking/annot/cuboid.h"
#include "tracking/phys/vis/visPhys.h"
#include "tracking/common/track.h"
#include "tracking/common/groupedTracks.h"

#include "tracking/common/mapper.h"
#include "tracking/vis/visualizer.h"

#include "ceres/impl/functorInfo.hpp"
#include "ceres/ceresUtil.h"
#include "ceres/rotation.h"

#include "ceres/ceres.h"
#include <cmath>
#include <cstdio>
#include <iostream>
#include <array>
#include "tracking/phys/physProblem.h"


namespace tracking {
    namespace bundle_physics {
        template<typename _Scalar>
        void BundleWithPhysics::qToRotationMatrix(Eigen::Matrix<_Scalar, 3, 3> &R, const ceres::CeresScalar q[4]) {
            QuaternionT pose;
            ceresToQuaternion(q, pose);
            R = pose.toRotationMatrix();
        }
    } //...ns bundle_phsyics
} //...ns tracking

int tracking::bundle_physics::BundleWithPhysics::animateCuboids2(
    BundleWithPhysicsResult       &         out,
    CuboidsT                 const&         cuboids,
    GroupedTracks2d          const&         tracks2d,
    //LinRgbsT                 const&         rgbs,
    FrameIdsT                const&         frameIds,
    Mapper                   const&         mapper,
    Weights                  const&         weights,
    BundleWithPhysicsResult  const* const   initial,
    bool                     const          use2dParabolaTerms,
    std::string              const          showFlags
) {
    int ret = EXIT_SUCCESS;
    PhysProblem pp = setup(cuboids, tracks2d, /*rgbs,*/ frameIds, mapper, weights, initial, use2dParabolaTerms);

    // =========================== Run the solver =========================== //
    if (EXIT_SUCCESS == ret) {
        out = solve(pp.getProblem(), pp.getIndexer(), pp.getCostFunctors(),
                    cuboids, frameIds, weights, initial, pp.getConsts(), pp.getParticipants(), pp.getPoseLoss());

        if (out.terminationType != ceres::FAILURE)
            bundle_physics::output(out, cuboids, tracks2d, frameIds, pp.getIndexer(), pp.getConsts(), /* doVis: */ false);
        else
            ret = EXIT_FAILURE;
    }

    // vis
    if (weights.doVis && EXIT_SUCCESS == ret) {
        showPhysOutput(out, cuboids, pp.getIndexer(), pp.getConsts(), showFlags);
    } //...vis

    return EXIT_SUCCESS;
} //...animateCuboids2

/** \param[in] problem Can use the provided problem, if passed. */
auto tracking::bundle_physics::BundleWithPhysics::setup(
    CuboidsT                 const&         cuboids,
    GroupedTracks2d          const&         tracks2d,
    //LinRgbsT                 const&         rgbs,
    FrameIdsT                const&         frameIds,
    Mapper                   const&         mapper,
    Weights                  const&         weights,
    BundleWithPhysicsResult  const* const   initial,
    bool                     const          use2dParabolaTerms,
    std::shared_ptr<ceres::Problem>         problemArg
) -> PhysProblem {
    using ceres::FunctorInfo;
    using ceres::CostFunction;
    using ceres::CeresVector3;
    using ceres::CeresVector4;
    using ceres::CeresMatrix3;
    using ceres::MapCeresVector3;
    using ceres::MapConstCeresVector3;
    using ceres::MapCeresVector4;
    using ceres::MapConstCeresVector4;
    using Soup::vis::Visualizer;

    if (frameIds.size() > 3) {
        std::cerr << "expecting 3 frameIds: start, collision,end" << std::endl;
    }
    if ( weights.initFlags && !initial ) {
        std::cerr << "need initial not to be nullptr for initFlags>0" << std::endl;
        throw new Weights_InitDiscrepancyException("");
    }

    std::cout << "[" << __func__ << "] " << "weights:\n";
    std::cout << "\tpos: " << weights.observedPosWeight << "\n"
              << "\tori: " << weights.observedPoseWeight << "\n"
              << "\timpulse:" << weights.velocityWeight << "\n"
              << "\tconserv:" << weights.conservationWeight << "\n"
              << "\tCor:"  << weights.corWeight << "\n"
              << "\tKE:"  << weights.keWeight << "\n";

    PhysProblem pp(problemArg);
    pp.setConsts(new Consts{weights.fps, frameIds});
    Consts const& consts = pp.getConsts();

    std::vector<CuboidId> const& participants = pp.getParticipants();

    // Init indexer
    pp.setIndexer(new PhysIndexer{frameIds, cuboids.size()});
    PhysIndexer &indexer = pp.getIndexer();

    for (auto const &track2d : tracks2d)
        indexer.addPoint(track2d.getLabel());
    indexer.allocate();

    ceres::LossFunctionWrapper *const poseLoss(
        new ceres::LossFunctionWrapper(
            new ceres::ScaledLoss(NULL,1.,ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP) );
    pp.setPoseLoss(poseLoss);

    std::cout << "[" << __func__ << "] " << pp.getProblemPtrRef().use_count() << std::endl;
    ceres::Problem       &problem      = pp.getProblem();
    std::cout << "[" << __func__ << "] " << pp.getProblemPtrRef().use_count() << std::endl;
    ceres::FunctorInfosT &costFunctors = pp.getCostFunctors();

    std::map<int, // cuboidId
        std::map<CollId, // collisionId
            std::pair<const Cuboid::StatesT::value_type *, // frameIdAndPoseLoc before
                const Cuboid::StatesT::value_type *  // frameIdAndPoseLoc after
            > > > closestPoses;

    if (weights.observedPoseWeight > 0.)
        addPoseTerms( problem, indexer, costFunctors, frameIds, weights, cuboids, consts, poseLoss );
    if (weights.observedPosWeight > 0.)
        addParabolaTerms(problem, indexer, costFunctors, frameIds, weights, mapper, cuboids, consts, initial,
                         use2dParabolaTerms);

    for (CuboidId cuboidId = 0; cuboidId != CuboidId(cuboids.size()); ++cuboidId) {
        const Cuboid &cuboid(cuboids.at(cuboidId));

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
                    if (collId < partId) { // after part
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
            mass[0] = 1.;
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

    if (weights.initFlags & Weights::USE_INPUT_MASS) {
        CeresScalar *mass = indexer.getMass(1, nullptr);
        mass[0] = initial->mass;
        std::cout << "[" << __func__ << "] " << "set initial mass ratio to " << mass[0] << std::endl;
        problem.SetParameterBlockConstant(mass);
    }

    // init poses and time
    for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
        const int partIdBefore = getPartIdBefore( collId );
        const int partIdAfter  = getPartIdAfter ( collId );
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
            std::cout << "collisionTime inited to " << collisionTime[0] << ", and bounded:" << frameIds.at(collId + 1)
                      << ".." << getNext(frameIds.at(collId + 1)) << std::endl;
        } //...init collision time

        // init poses
        for (CuboidId cuboidId = 0; cuboidId != static_cast<CuboidId>(cuboids.size()); ++cuboidId) {
            const Cuboid &cuboid = cuboids.at(cuboidId);
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
                std::cout << "prevTime:" << prevTime << ", coll: " << collisionTime[0] << ", nextTime:" << nextTime
                          << ", timeElapsed: " << timeElapsed << std::endl;
                throw new BundleWithPhysics_InitPoses_CollTimeNotBetweenObservationsException("");
            }

            const QuaternionT &prevPose = closest.first->second.getPose();
            const QuaternionT &nextPose = closest.second->second.getPose();
            const QuaternionT initPose  = prevPose.slerp((collisionTime[0] - prevTime) / timeElapsed, nextPose);
            if (weights.initFlags & Weights::USE_INPUT_POSES) {
                if ( !initial )
                    throw new BundleWithPhysics_InitPoses_NoInitProvidedException("");
                if ( initial->collisions.size() > 1 )
                    throw new BundleWithPhysics_InitPoses_ASingleCollisionAssumedException("");
                std::cout << "initing pose to " << MapConstCeresVector4( poseAddress ).transpose() << std::endl;
                quaternionToCeres( initial->collisions.at(0).states.first.at(cuboidId).getPose(), poseAddress );
                std::cout << "inited pose to " << MapConstCeresVector4( poseAddress ).transpose() << std::endl;
            }
            else
                quaternionToCeres(initPose, poseAddress);

            if (weights.fixFlags & Weights::FIX_POSE) {
                std::cout << "fixing pose for " << cuboidId << " and coll " << collId << std::endl;
                problem.SetParameterBlockConstant( poseAddress );
                std::cout << "fixed pose" << std::endl;
            }

            // initialize momentum
            if (initial && (weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_MOMENTUM) ) {
                if ( initial->hasMomentum(cuboidId,partIdBefore) ) {
                    CeresScalar* const momentumBefore = indexer.getMomentum(cuboidId, partIdBefore);
                    (MapCeresVector3(momentumBefore)) = initial->momenta.at(cuboidId).at(partIdBefore).cast<CeresScalar>();
                    if ( weights.fixFlags & Weights::FIX_FLAGS::FIX_MOMENTA ) {
                        std::cout << "[" << __func__ << "] " << "fixing momentum[" << cuboidId << "][" << partIdBefore << "]" << std::endl;
                        problem.SetParameterBlockConstant(momentumBefore);
                    }
                }
                if ( initial->hasMomentum(cuboidId,partIdAfter) ) {
                    CeresScalar* const momentumAfter = indexer.getMomentum(cuboidId, partIdAfter);
                    (MapCeresVector3(momentumAfter)) = initial->momenta.at(cuboidId).at(partIdAfter).cast<CeresScalar>();
                    if ( weights.fixFlags & Weights::FIX_FLAGS::FIX_MOMENTA ) {
                        problem.SetParameterBlockConstant(momentumAfter);
                        std::cout << "[" << __func__ << "] " << "fixing momentum[" << cuboidId << "][" << partIdAfter << "]" << std::endl;
                    }
                }
                //throw new BundleWithPhysics_InitMomentum_UseInputMomentumNotImplementedException("");
            } else {
                CeresScalar  *mass = indexer.getMass(cuboidId);
                std::cout << "mass of cuboid " << cuboidId << ": " << mass[0] << std::endl;
                CeresVector3 initialI;
                cuboid.getIFromMass(initialI, mass[0]);
                std::cout << "I of cuboid " << cuboidId << ": " << initialI.transpose() << std::endl;
                const CeresMatrix3 R = initPose.toRotationMatrix().cast<CeresScalar>();
                std::cout << "R:\n" << R << std::endl;
                CeresMatrix3 currI = R * initialI.asDiagonal() * R.transpose();
                std::cout << "currI of cuboid " << cuboidId << ":\n" << currI << std::endl;

                const QuaternionT omega0 = getEmpOmega(prevPose, initPose, Scalar((collisionTime[0]-prevTime) * consts.k_dt));
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
                    << ", from omega "  << omega1.coeffs().transpose()
                    << std::endl;
                }
                QuaternionT prev( QuaternionT::Unit() );
                QuaternionT q1( Eigen::AngleAxisf( M_PI_2, Eigen::Vector3f::UnitX()) );
                QuaternionT q2( Eigen::AngleAxisf(-M_PI_2,-Eigen::Vector3f::UnitX()) );
//                std::cout << "q1: " << q1.coeffs().transpose() << std::endl;
//                std::cout << "q2: " << q2.coeffs().transpose() << std::endl;
//                std::cout << "getEmp:" << getEmpOmega( prev, q1, 1.f ).coeffs().transpose() << std::endl;
//                std::cout << "getEmp:" << getEmpOmega( prev, q2, 1.f ).coeffs().transpose() << std::endl;
#if 0
                // debug init momentum
                {
                    Soup::vis::Visualizer<Scalar> vis( cuboids.at(cuboidId).getName() );

                    const CeresScalar *const massAddress     = indexer.getMass( cuboidId );
                    const CeresScalar timeElapsed0 = (collisionTime[0]-prevTime) * consts.k_dt;
                    std::cout << "timeElapsed0: " << timeElapsed0 << ", between " << collisionTime[0] << ", and " << prevTime << ", with consts.k_dt: "<< consts.k_dt << std::endl;
                    QuaternionT tmpPose = prevPose;
                    Vector3 x = -Vector3::Ones();
#warning TODO: change I
                    CeresVector3 invI;
                    //CeresVector3 sqrSize = (cuboid.getSize().array() * cuboid.getSize().array()).cast<CeresScalar>();
                    //getInvIBox( mass, sqrSize.data(), invI );
                    cuboid.getInverseIFromMass( invI, mass[0] );
                    std::cout << "invI:" << invI.transpose() << std::endl;
                    drawCuboid( vis, TranslationT(x) * prevPose.toRotationMatrix() * cuboid.getSizeTransform(), Vector3(.2, .8, .2), "prevPose" );

                    const int split = 10;
                    const CeresScalar *const prevMomentumAddress = indexer.getMomentum(cuboidId, getPartIdBefore(collId) );
                    std::cout << "prevMomentum: " << (MapConstCeresVector3(prevMomentumAddress)).transpose() << std::endl;
                    std::cout << "prevPose: " << prevPose.coeffs().transpose() << std::endl;
                    for ( int i = 0; i != split; ++i )
                    {
                        sprintf( name, "tmpPose%d", i );
                        x+= Vector3( 0.1,0.1,0.1 );
                        CeresScalar ceresTmpPose[4], qTmp[4];
                        quaternionToCeres( tmpPose, ceresTmpPose );
                        integrateQuaternion( ceresTmpPose, prevMomentumAddress, timeElapsed0/float(split)/2., invI.data(), qTmp, 10 );
                        ceresToQuaternion( qTmp, tmpPose );
                        std::cout << "tmpPose[" << i  << "]:" << tmpPose.coeffs().transpose() << std::endl;
                        drawCuboid(vis, TranslationT(x) * tmpPose.toRotationMatrix() * cuboid.getSizeTransform(), Vector3(.2, .2, .2), name );
                    }

                    drawCuboid( vis, TranslationT(Vector3::Zero()) * initPose.toRotationMatrix() * cuboid.getSizeTransform(), Vector3(.8, .2, .2), "initPose" );

                    const CeresScalar timeElapsed1 = (nextTime-collisionTime[0]) * consts.k_dt;
                    const CeresScalar *const nextMomentumAddress = indexer.getMomentum(cuboidId, getPartIdAfter(collId) );
                    tmpPose = initPose;
                    std::cout << "nextMomentum: " << (MapConstCeresVector3(nextMomentumAddress)).transpose() << std::endl;
                    std::cout << "timeElapsed1: " << timeElapsed1 << ", between " << collisionTime[0] << ", and " << nextTime << ", with consts.k_dt: "<< consts.k_dt << std::endl;
                    std::cout << "initPose: " << initPose.coeffs().transpose() << std::endl;
                    for ( int i = 0; i != split; ++i )
                    {
                        sprintf( name, "tmpPose%d", split+i );
                        x+= Vector3( 0.1,0.1,0.1 );
                        CeresScalar ceresTmpPose[4], qTmp[4];
                        quaternionToCeres( tmpPose, ceresTmpPose );
                        integrateQuaternion( ceresTmpPose, nextMomentumAddress, timeElapsed1/float(split)/2., invI.data(), qTmp, 10 );
                        ceresToQuaternion( qTmp, tmpPose );
                        std::cout << "tmpPose[" << split + i  << "]:" << tmpPose.coeffs().transpose() << std::endl;
                        drawCuboid(vis, TranslationT(x) * tmpPose.toRotationMatrix() * cuboid.getSizeTransform(), Vector3(.2, .2, .2), name );
                    }

                    drawCuboid( vis, TranslationT(Vector3::Ones()) * nextPose.toRotationMatrix() * cuboid.getSizeTransform(), Vector3(.2, .8, .2), "nextPose" );
                    std::cout << "nextPose: " << nextPose.coeffs().transpose() << std::endl;
                    vis.spin();
                }
#endif
            } //...init momentum
        } //...for cuboids
    } //...for collisions

    if (weights.conservationWeight > 0.)
        addConservationTerms(problem, indexer, costFunctors, frameIds, weights, cuboids, participants, consts);

    if (weights.gravityDownWeight > 0.)
        addGravityTerm(problem, indexer, costFunctors, weights, consts);

    std::cout << "[" << __func__ << "] before return..." << pp.getProblemPtrRef().use_count() << std::endl;
    return std::move(pp);
} //...animateCuboids2

ceres::Solver::Options tracking::bundle_physics::BundleWithPhysics::getSolverOptions() {
    ceres::Solver::Options options;
    options.linear_solver_type           = ceres::SPARSE_NORMAL_CHOLESKY; // nonlin lsq
    //options.linear_solver_type           = ceres::CGNR; // nonlin lsq
    options.preconditioner_type          = ceres::JACOBI;
    //options.dynamic_sparsity = true;
//        options.minimizer_progress_to_stdout = weights.doVis;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations           = 4000;
    options.num_threads                  = 16;
    //options.max_solver_time_in_seconds   = 30;
    options.function_tolerance           = 1.e-12; // 16
    options.parameter_tolerance          = 1.e-12; // 16
    options.gradient_tolerance           = options.function_tolerance * 1.e-4;
    //std::cout << "linsearch: " << options.line_search_direction_type << std::endl;
    //options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    //std::cout << "linsearch: " << options.line_search_direction_type << std::endl;
    return options;
}

auto tracking::bundle_physics::BundleWithPhysics::solve(
    ceres::Problem               &       problem,
    PhysIndexer                  &       indexer,
    ceres::FunctorInfosT         &       costFunctors,
    CuboidsT                const&       cuboids,
    FrameIdsT               const&       frameIds,
    Weights                 const&       weights,
    BundleWithPhysicsResult const* const initial,
    Consts                  const&       consts,
    std::vector<CuboidId>   const&       participants,
    ceres::LossFunctionWrapper   * const poseLoss) -> BundleWithPhysicsResult {

    ceres::Solver::Options options = BundleWithPhysics::getSolverOptions();

    BundleWithPhysicsResult out;
    const tracking::CollId collId(0);
    // solve
    out.getLog().clear();
    //out.energies.clear();
    //out.runTimes.clear();
    //out.massEstimates.clear();
    ceres::Solver::Summary summary;
    if (!(weights.solveFlags & Weights::NO_SOLVE)) {
        // ================== SOLVE 1 ================== //
        if (weights.solveStages & Weights::SOLVE_MOMENTA) {
            std::cout << __FILE__ << ":" << __LINE__ << " solving without impulse terms...\n";
            ceres::Solve(options, &problem, &summary);
            if (weights.doVis) {
                cout << summary.FullReport() << "\n";
                printFunctors(costFunctors);
            }
        }
        out.getLog().log(summary, indexer, collId, participants);
//        out.energies.push_back(summary.initial_cost );
//        out.energies.push_back(summary.final_cost );
//        out.runTimes.push_back(summary.total_time_in_seconds );
//        out.massEstimates.push_back(*indexer.getMassConst(getParticipant(1, collId, participants)));

        CuboidId const objA = getParticipant(0, collId, participants);
        CuboidId const objB = getParticipant(1, collId, participants);

        // This is not supposed to do anything:
        if (problem.IsParameterBlockConstant(indexer.getParabolaTranslation(objA, collId)))
            std::cerr << "[" << __func__ << "] " << "whyyy..." << std::endl;

        problem.SetParameterBlockVariable(indexer.getParabolaTranslation(objA, collId));
        problem.SetParameterBlockVariable(indexer.getParabolaTranslation(objB, collId));
        problem.SetParameterBlockVariable(indexer.getParabolaFreeParams(objA, getPartIdBefore(collId)));
        problem.SetParameterBlockVariable(indexer.getParabolaFreeParams(objA, getPartIdAfter(collId)));
        problem.SetParameterBlockVariable(indexer.getParabolaFreeParams(objB, getPartIdBefore(collId)));
        problem.SetParameterBlockVariable(indexer.getParabolaFreeParams(objB, getPartIdAfter(collId)));
        if (weights.solveStages & Weights::SOLVE_MOMENTA) {
            std::cout << __FILE__ << ":" << __LINE__ << " fixing mass...\n";
            problem.SetParameterBlockConstant(indexer.getMass(objB));
        }

//        std::cout << __FILE__ << ":" << __LINE__ << " limiting momenta..\n";
//        for (PartId partId = 0; partId != 1; ++partId)
//            for (int dim = 0; dim != 3; ++dim) {
//                problem.SetParameterLowerBound(indexer.getMomentum(objA, partId), dim, -.2);
//                problem.SetParameterLowerBound(indexer.getMomentum(objB, partId), dim, -.2);
//                problem.SetParameterUpperBound(indexer.getMomentum(objA, partId), dim,  .2);
//                problem.SetParameterUpperBound(indexer.getMomentum(objB, partId), dim,  .2);
//            }

        std::cout << "adding impulse terms" << std::endl;
        addImpulseTerms(problem, indexer, costFunctors, frameIds, weights, cuboids, participants, consts);
        addCorFunctor(problem, indexer, costFunctors, frameIds, weights, cuboids.at(0), cuboids.at(1), consts);
        initCollPoint(problem, indexer, weights, participants, consts, initial);
        poseLoss->Reset(new ceres::ScaledLoss(NULL, 10., ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP);

        // ================== SOLVE 2 ================== //
        if (weights.solveStages & tracking::bundle_physics::Weights::SOLVE_COUPLED) {
            std::cout << __FILE__ << ":" << __LINE__ << " solving with impulse terms...\n";
            ceres::Solve(options, &problem, &summary);
        }

        out.getLog().log(summary, indexer, collId, participants);
//        out.energies.push_back( summary.final_cost );
//        out.runTimes.push_back(summary.total_time_in_seconds );
//        out.massEstimates.push_back( *indexer.getMass(getParticipant(1, collId, participants)) );

        cout << "solve2" << endl;
        problem.SetParameterBlockVariable(indexer.getCollisionPoint(collId));

        // ================== SOLVE 3 ==================
        if (weights.solveStages & tracking::bundle_physics::Weights::SOLVE_FREE_CP) {
            std::cout << __FILE__ << ":" << __LINE__ << " solving free collision point...\n";
            ceres::Solve(options, &problem, &summary);
        }
        out.getLog().log(summary, indexer, collId, participants);
        //out.energies.push_back(summary.final_cost );
        //out.runTimes.push_back(summary.total_time_in_seconds );
        //out.massEstimates.push_back(*indexer.getMassConst(getParticipant(1, collId, participants)) );

        // report
        if (weights.doVis) {
            cout << summary.FullReport() << "\n";
            printFunctors(costFunctors);
        }

        if  (weights.solveStages & tracking::bundle_physics::Weights::SOLVE_FREE_MASS) {
            problem.SetParameterBlockConstant(indexer.getMomentum(objA, getPartIdBefore(collId)));
            problem.SetParameterBlockConstant(indexer.getMomentum(objA, getPartIdAfter(collId)));
            problem.SetParameterBlockConstant(indexer.getMomentum(objB, getPartIdBefore(collId)));
            problem.SetParameterBlockConstant(indexer.getMomentum(objB, getPartIdAfter(collId)));
            problem.SetParameterBlockVariable(indexer.getMass(objB));
            ceres::Solve(options, &problem, &summary);
            out.getLog().log(summary, indexer, collId, participants);
            //out.energies.push_back(summary.final_cost );
            //out.runTimes.push_back(summary.total_time_in_seconds );
            //out.massEstimates.push_back(*indexer.getMass(getParticipant(1, collId, participants)));
        }
    }

    // check for failure
    if (!(weights.solveFlags & tracking::bundle_physics::Weights::NO_SOLVE)) {
        out.terminationType = summary.termination_type;
        if (summary.termination_type == ceres::FAILURE) {
            std::cerr << "[" << __func__ << "] " << "!!!!!!!!! failure..."<< std::endl;
        }
    } else
        out.terminationType = ceres::NO_CONVERGENCE;

    return out;
} //...solve

