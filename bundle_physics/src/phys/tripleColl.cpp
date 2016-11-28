//
// Created by bontius on 07/07/16.
//

#include <tracking/phys/vis/visPhys.h>
#include "tracking/phys/partUtil.h"
#include "tracking/phys/energyTerms/CoRBoundsTerm.h"
#include "tracking/phys/energyTerms/impulseTerm.h"
#include "tracking/phys/tripleColl.h"

#include "tracking/phys/bundleWithPhysics.h"
#include "tracking/phys/physProblem.h"
#include "tracking/phys/physOutput.h"
#include "ceres/impl/functorInfo.hpp"
#include "tracking/common/groupedTracks.h"

#include "tracking/annot/cuboid.h"

#include <tracking/phys/energyTerms/parabolaTerm.h>
namespace tracking {
namespace bundle_physics {

struct EqualPosFunctor {
    public:
        enum {NUM_RESIDUALS = 3};

        EqualPosFunctor(FrameId const frameId, ceres::CeresScalar const sqrtWeight = 1.)
            : _time(frameId), _sqrtWeight(sqrtWeight) {}

        template <typename T>
        bool operator()(T const rotG[2], T const a[1],
                        T const translation0[3], T const free0[3], T const collTime0[1],
                        T const translation1[3], T const free1[3], T const collTime1[1],
                        T *residuals) const {
            T x0[3];
            ParabolaCostFunctor::getPositionAtTime(x0, rotG, a, translation0, free0, T(_time) - collTime0[0]);
            T x1[3];
            ParabolaCostFunctor::getPositionAtTime(x1, rotG, a, translation1, free1, T(_time) - collTime1[0]);

            std::cout << "y0 0: " << ceres::printJet(free0[PhysIndexer::PARABOLA_FREE_ANGLE_OFFSET])
                      << "y0 1: " << ceres::printJet(free1[PhysIndexer::PARABOLA_FREE_ANGLE_OFFSET])
                                  << "\n";
            std::cout << "b1 0: " << ceres::printJet(free0[PhysIndexer::PARABOLA_FREE_S_OFFSET])
                      << "b1 1: " << ceres::printJet(free1[PhysIndexer::PARABOLA_FREE_S_OFFSET])
                      << "\n";

            std::cout << "b2 0: " << ceres::printJet(free0[PhysIndexer::PARABOLA_FREE_B_OFFSET])
                      << "b2 1: " << ceres::printJet(free1[PhysIndexer::PARABOLA_FREE_B_OFFSET])
                      << "\n";

            residuals[0] = T(_sqrtWeight) * (x0[0] - x1[0]);
            residuals[1] = T(_sqrtWeight) * (x0[1] - x1[1]);
            residuals[2] = T(_sqrtWeight) * (x0[2] - x1[2]);

            return true;
        }

        static ceres::CostFunction* Create(FrameId const frameId, ceres::CeresScalar const sqrtWeight = 1.) {
            return new ceres::AutoDiffCostFunction<EqualPosFunctor,NUM_RESIDUALS,2,1,3,3,1,3,3,1>{
                new EqualPosFunctor{ frameId, sqrtWeight } };
        }
    protected:
        ceres::CeresScalar _time;       //!< observation time
        ceres::CeresScalar _sqrtWeight; //!< term weight
};

struct EqualPoseFunctor {
    public:
        enum {NUM_RESIDUALS = 3};

        EqualPoseFunctor(FrameId const frameId, ceres::CeresScalar const sqrtWeight = 1.)
            : _time(frameId), _sqrtWeight(sqrtWeight) {}

        template <typename T>
        bool operator()(T const rotG[2], T const a[1],
                        T const translation0[3], T const free0[3], T const collTime0[1],
                        T const translation1[3], T const free1[3], T const collTime1[1],
                        T *residuals) const {
            T x0[3];
            ParabolaCostFunctor::getPositionAtTime(x0, rotG, a, translation0, free0, T(_time) - collTime0[0]);
            T x1[3];
            ParabolaCostFunctor::getPositionAtTime(x1, rotG, a, translation1, free1, T(_time) - collTime1[0]);

            residuals[0] = T(_sqrtWeight) * (x0[0] - x1[0]);
            residuals[1] = T(_sqrtWeight) * (x0[1] - x1[1]);
            residuals[2] = T(_sqrtWeight) * (x0[2] - x1[2]);

            return true;
        }

        static ceres::CostFunction* Create(FrameId const frameId, ceres::CeresScalar const sqrtWeight = 1.) {
            return new ceres::AutoDiffCostFunction<EqualPoseFunctor,NUM_RESIDUALS,2,1,3,3,1,3,3,1>{
                new EqualPoseFunctor{ frameId, sqrtWeight } };
        }
#if 0
        template<typename _Scalar, typename _Vector4>
        static ceres::CostFunction *Create(double dt, const _Scalar *const size, const ceres::CeresScalar time, int const collTimeLowerBound, float const substepsPerFrame, SHAPE const shape, ceres::CeresScalar const sqrtWeight )
        {
            return (new ceres::AutoDiffCostFunction<PoseIntegralCostFunctor, NUM_RESIDUALS,
                                                    PhysIndexer::POSE_STRIDE, PhysIndexer::MOMENTUM_STRIDE, PhysIndexer::MASS_STRIDE, PhysIndexer::COLL_TIME_STRIDE>(
                new PoseIntegralCostFunctor(sqrtWeight, dt, size, observedPose, queryFrameId, collTimeLowerBound, substepsPerFrame, shape)));
        }
#endif
    protected:
        ceres::CeresScalar _time;       //!< observation time
        ceres::CeresScalar _sqrtWeight; //!< term weight
};

}; //...ns bundle_phsyics
} //...ns tracking

int tracking::bundle_physics::tripleColl(
    BundleWithPhysicsResult      &       out,
    CuboidsT                const&       cuboids,
    GroupedTracks2d         const&       tracks2d,
    LinRgbsT                const&       rgbs,
    FrameIdsT               const&       frameIds,
    Mapper                  const&       mapper,
    Weights                 const&       weights,
    BundleWithPhysicsResult const* const initial,
    std::string             const        showFlags
) {
    int ret = EXIT_SUCCESS;

    std::cout << "[" << __func__ << "] " << "got " << cuboids.size() << " cuboids" << std::endl;

    for (Cuboid const& cuboid : cuboids) {
        std::cout << "[" << __func__ << "] " << cuboid.getName();
        int positions(0), poses(0);
        for (auto const& state : cuboid.getStates()) {
            positions += state.second.hasPos();
            poses += state.second.hasPose();
        }
        std::cout << " has " << positions << " positions and " << poses << " poses" << std::endl;
    }

    if (frameIds.size() != 4) {
        std::cerr << "[" << __func__ << "] " << "need 4 frameIds for the 2 collisions" << std::endl;
        return EXIT_FAILURE;
    }

    if (initial) {
        std::cerr << "[" << __func__ << "] " << "TODO: implement initialization" << std::endl;
    }


    FrameIdsT frameIds0(3);
    std::copy(frameIds.begin(), frameIds.begin() + 3, frameIds0.begin());
    CuboidsT cuboids0 { cuboids.at(0), cuboids.at(1) };
    BundleWithPhysicsResult initial0, initial1;
    if (initial) {
        if (initial->collTimes.find(0) != initial->collTimes.end())
            initial0.collTimes[0] = initial->collTimes.at(0);
        initial0.rotX = initial->rotX;
        initial0.a = initial->a;
        initial0.rotY1 = initial->rotY1;
        initial0.parabolas = initial->parabolas;

        if (initial->collTimes.find(1) != initial->collTimes.end())
            initial1.collTimes[0] = initial->collTimes.at(1);
        initial1.rotX = initial->rotX;
        initial1.a = initial->a;
        initial1.rotY1 = initial->rotY1;
        initial1.parabolas[0] = initial->parabolas.at(3);
        initial1.parabolas[1] = initial->parabolas.at(2);
    }
    std::cout << "[" << __func__ << "] collision of " << cuboids0.at(0).getName() << " (=?0) and " << cuboids0.at(1).getName() << " (=?1)" << std::endl;
    PhysProblem pp0 = BundleWithPhysics::setup(cuboids0, tracks2d, /*rgbs,*/ frameIds0, mapper, weights, &initial0,
        /* use2d terms: */ true, nullptr);

    FrameIdsT frameIds1(3);
    std::copy(frameIds.begin()+1, frameIds.end(), frameIds1.begin());
    step(frameIds1.front()); // upper bound
    CuboidsT cuboids1 { cuboids.at(0), cuboids.at(2) };
    std::cout << "[" << __func__ << "] collision of " << cuboids1.at(0).getName() << " (=?0) and " << cuboids1.at(1).getName() << " (=?2)" << std::endl;
    Weights weights1 = weights;
    weights1.bounds.collTime.lower = static_cast<double>(        frameIds1.at(1))  - (weights1.fps / 60. - 1.);
    weights1.bounds.collTime.upper = static_cast<double>(getNext(frameIds1.at(1))) + (weights1.fps / 60. - 1.);
    PhysProblem pp1 = BundleWithPhysics::setup(cuboids1, tracks2d, /*rgbs,*/ frameIds1, mapper, weights1, &initial1,
          /* use2d terms: */ true, pp0.getProblemPtr());
    assert(&pp0.getProblem() == &pp1.getProblem());

    CollId const collId0 = 0, collId1 = 0;
    CuboidId c0 = getParticipant(0,collId0,pp0.getParticipants());
    CuboidId c1 = getParticipant(0,collId1,pp1.getParticipants());
    double const equalWeight = 10.;
    for (FrameId frameId = getNext(frameIds.at(1)); frameId <= frameIds.at(2); step(frameId)) {
        ceres::CostFunction *costFunction = EqualPosFunctor::Create(frameId, equalWeight);
        std::vector<ceres::CeresScalar*> unknowns = {pp0.getIndexer().getParabolaRotationShared(),
                                                     pp0.getIndexer().getParabolaSquaredParam(),
                                                     pp0.getIndexer().getParabolaTranslation(c0,collId0),
                                                     pp0.getIndexer().getParabolaFreeParams(c0,1),
                                                     pp0.getIndexer().getCollisionTime(collId0),
                                                     pp1.getIndexer().getParabolaTranslation(c1, /* collId: */ collId1),
                                                     pp1.getIndexer().getParabolaFreeParams(c1,  /* partId: */ 0),
                                                     pp1.getIndexer().getCollisionTime(/* collId: */ collId1)};
        pp0.getProblem().AddResidualBlock(costFunction, nullptr,
                                          unknowns.at(0), unknowns.at(1), unknowns.at(2), unknowns.at(3),
                                          unknowns.at(4), unknowns.at(5), unknowns.at(6), unknowns.at(7));
        char name[255];
        sprintf(name, "Equal_%s_%s_time%u", cuboids0[0].getName().c_str(), cuboids1[0].getName().c_str(), static_cast<unsigned>(frameId));
        pp0.getCostFunctors().emplace_back(ceres::FunctorInfo{name, costFunction, unknowns});
        pp1.getCostFunctors().emplace_back(ceres::FunctorInfo{name, costFunction, unknowns});
    }

    std::vector<ceres::ResidualBlockId> blocks0;
    pp1.getProblem().GetResidualBlocksForParameterBlock(pp0.getIndexer().getCollisionTime(collId0), &blocks0);
    std::vector<ceres::ResidualBlockId> blocks1;
    pp1.getProblem().GetResidualBlocksForParameterBlock(pp1.getIndexer().getCollisionTime(collId1), &blocks1);
    std::cout << "[" << __func__ << "] " << "time address0: "
              << pp0.getIndexer().getCollisionTime(collId0) << ","
              << pp1.getIndexer().getCollisionTime(collId1)
              << ", usages: " << blocks0.size() << "," << blocks1.size()
              << std::endl;



    BundleWithPhysicsResult out1;
    auto options = BundleWithPhysics::getSolverOptions();
    options.max_num_iterations = 1000;
    ceres::Solver::Summary summary;
    std::cout << "[" << __func__ << "] " << "solve0" << std::endl;
    if (!(weights.solveFlags & Weights::NO_SOLVE)) {
        ceres::Solve(options, &pp0.getProblem(), &summary);
        out.getLog().log(summary, pp0.getIndexer(), collId0, pp0.getParticipants());
        out1.getLog().log(summary, pp1.getIndexer(), collId0, pp1.getParticipants());
        if (weights.doVis) {
            std::cout << summary.FullReport() << "\n";
            //printFunctors(pp0.getCostFunctors());
            //printFunctors(pp1.getCostFunctors());
        }
    } else
        std::cout << "[" << __func__ << "] " << "no solve" << std::endl;

    CuboidId const objA0 = getParticipant(0, collId0, pp0.getParticipants());
    CuboidId const objB0 = getParticipant(1, collId0, pp0.getParticipants());
    CuboidId const objA1 = getParticipant(0, collId1, pp1.getParticipants());
    CuboidId const objB1 = getParticipant(1, collId1, pp1.getParticipants());
    ceres::Problem &problem = pp0.getProblem();

//    if (weights.solveStages & Weights::SOLVE_MOMENTA) {
//        problem.SetParameterBlockConstant(pp0.getIndexer().getMass(objB0));
//        problem.SetParameterBlockConstant(pp1.getIndexer().getMass(objB1));
//    }
    std::cout << "adding impulse terms" << std::endl;
    addImpulseTerms(pp0.getProblem(), pp0.getIndexer(), pp0.getCostFunctors(), frameIds0, weights, cuboids0, pp0.getParticipants(), pp0.getConsts());
    addImpulseTerms(pp1.getProblem(), pp1.getIndexer(), pp1.getCostFunctors(), frameIds1, weights1, cuboids1, pp1.getParticipants(), pp1.getConsts());
    addCorFunctor(pp0.getProblem(), pp0.getIndexer(), pp0.getCostFunctors(), frameIds0, weights, cuboids0.at(0), cuboids0.at(1), pp0.getConsts());
    addCorFunctor(pp1.getProblem(), pp1.getIndexer(), pp1.getCostFunctors(), frameIds1, weights1, cuboids1.at(0), cuboids1.at(1), pp1.getConsts());
    initCollPoint(pp0.getProblem(), pp0.getIndexer(), weights, pp0.getParticipants(), pp0.getConsts(), initial);
    initCollPoint(pp1.getProblem(), pp1.getIndexer(), weights1, pp1.getParticipants(), pp1.getConsts(), initial);

    if (!(weights.solveFlags & Weights::NO_SOLVE)) {
        std::cout << "[" << __func__ << "] " << "solve1" << std::endl;
        ceres::Solve(options, &pp0.getProblem(), &summary);
        out.getLog().log(summary, pp0.getIndexer(), collId0, pp0.getParticipants());
        out1.getLog().log(summary, pp1.getIndexer(), collId0, pp1.getParticipants());
        if (weights.doVis) {
            std::cout << summary.FullReport() << "\n";
            //printFunctors(pp0.getCostFunctors());
            //printFunctors(pp1.getCostFunctors());
        }
    } else
        std::cout << "[" << __func__ << "] " << "no solve" << std::endl;

    // ================== SOLVE 3 ==================

    std::cout << "[" << __func__ << "] " << "solve1" << std::endl;
    problem.SetParameterBlockVariable(pp0.getIndexer().getCollisionPoint(collId0));
    problem.SetParameterBlockVariable(pp1.getIndexer().getCollisionPoint(collId1));
    if (!(weights.solveFlags & Weights::NO_SOLVE)) {
        ceres::Solve(options, &problem, &summary);
        out.getLog().log(summary, pp0.getIndexer(), collId0, pp0.getParticipants());
        out1.getLog().log(summary, pp1.getIndexer(), collId1, pp1.getParticipants());
        if (weights.doVis) {
            std::cout << summary.FullReport() << "\n";
            printFunctors(pp0.getCostFunctors());
            printFunctors(pp1.getCostFunctors());
        }
    }

    output(out, cuboids0, tracks2d, frameIds0, pp0.getIndexer(), pp0.getConsts(), /* doVis: */ false);
    // vis
    if (weights.doVis && EXIT_SUCCESS == ret) {
        showPhysOutput(out, cuboids0, pp0.getIndexer(), pp0.getConsts(), showFlags);
    } //...vis
    //createMovie(*out.cuboids, rgbs, frameIds0, mapper, &cuboids0, nullptr, &tracks2d, &out.gravity,
    //            weights.fps);

    output(out1, cuboids1, tracks2d, frameIds1, pp1.getIndexer(), pp1.getConsts(), /* doVis: */ false);
    if (weights.doVis && EXIT_SUCCESS == ret) {
        showPhysOutput(out1, cuboids1, pp1.getIndexer(), pp1.getConsts(), showFlags);
    } //...vis

    LinRgbsT rgbs1(rgbs.begin() + frameIds1.front(), rgbs.begin() + frameIds1.back() + 1);
    createMovie(*out1.cuboids, rgbs1, frameIds1, mapper, &cuboids1, nullptr, &tracks2d, &out1.gravity,
                weights.fps);
#if 1
    for (auto const& cuboid : *out1.cuboids) {
        auto iter = std::find_if(out.cuboids->begin(), out.cuboids->end(), [cuboid](Cuboid const& cuboid1) {
            return cuboid1.getName() == cuboid.getName();
        });
        if (out.cuboids->end() == iter)
            out.cuboids->push_back(cuboid);
        else {
            for (auto const& entry : cuboid.getStates()) {
                if (!iter->hasFrame(entry.first)) {
                    iter->setPosition(entry.first, entry.second.getPosition());
                    iter->setPose(entry.first, entry.second.getPose());
                }
            }
        }
    }
#endif

    return 1;
} //...tripleColl()