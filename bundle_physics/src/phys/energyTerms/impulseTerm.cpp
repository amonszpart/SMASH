//
// Created by bontius on 04/01/16.
//

#include "tracking/phys/energyTerms/impulseTerm.h"
#include "tracking/annot/cuboid.h"

#include "tracking/phys/energyTerms/impl/impulseTerm.hpp"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/partUtil.h"
#include <vector>

namespace tracking {
    namespace bundle_physics {
        void addImpulseTerms(
            ceres::Problem              & problem,
            PhysIndexer                 & indexer,
            ceres::FunctorInfosT        & costFunctors,
            const FrameIdsT             & frameIds,
            const Weights               & weights,
            const CuboidsT              & cuboids,
            const std::vector<CuboidId> & participants,
            const Consts                & consts
        ) {
            using ceres::CeresScalar;
            using ceres::CostFunction;

            char name[512];

            //CeresScalar *const parabolaShared = indexer.getParabolaSharedParams();
            CeresScalar *const parabolaRotShared = indexer.getParabolaRotationShared();
            CeresScalar *const parabolaA         = indexer.getParabolaSquaredParam();
            for ( CuboidId cuboidId = 0; cuboidId != CuboidId(cuboids.size()); ++cuboidId ) {
                const Cuboid &cuboid = cuboids.at(cuboidId);
                if ( !cuboid.isMassFinite() )
                    throw new BundleWithPhysics_ImpulseTerms_InfiniteMassException("");

                // transitions: 1 transition for 3 frameIds (beg, just before collision, end)
                for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
                    //CeresScalar *const collisionTime = indexer.getCollisionTime(collId);
                    CeresScalar *const parabolaFree0 = indexer.getParabolaFreeParams(cuboidId, getPartIdBefore(collId));
                    CeresScalar *const parabolaFree1 = indexer.getParabolaFreeParams(cuboidId, getPartIdAfter(collId));

                    // velocity
                    CeresScalar *const mass    = indexer.getMass(cuboidId);
                    CeresScalar *const impulse = indexer.getImpulse(collId);
                    {
                        CostFunction *const costFunction = LinVelCostFunctor::Create(weights.velocityWeight, consts.k_fps, frameIds.front(), cuboidId);
                        if (weights.velocityWeight > 0.) {
                            problem.AddResidualBlock(costFunction, NULL, parabolaRotShared, parabolaA, parabolaFree0, parabolaFree1, impulse, mass);
                        }

                        sprintf(name, "VelocityCostFunctor_cub%d_coll%u", cuboidId, collId);
                        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {parabolaRotShared, parabolaA, parabolaFree0, parabolaFree1, impulse, mass}));
                    }

                    // angVel exchange
                    const CuboidId other = getOther(cuboidId, participants);
                    std::cout << "other of cuboid" << cuboidId << ", is cuboid" << other << std::endl;
                    CeresScalar *const parabolaTranslationThis  = indexer.getParabolaTranslation(cuboidId, collId);
                    CeresScalar *const parabolaTranslationOther = indexer.getParabolaTranslation(other, collId);
                    //CeresScalar *const poseAddress              = indexer.getCollisionPose(cuboidId, collId);
                    CeresScalar *const momentum0                = indexer.getMomentum(cuboidId, getPartIdBefore(collId));
                    CeresScalar *const momentum1                = indexer.getMomentum(cuboidId, getPartIdAfter(collId));
                    CeresScalar *const collPoint                = indexer.getCollisionPoint(collId);
                    {
                        CostFunction *const costFunction = AngVelCostFunctor::Create(weights.velocityWeight, cuboid.getSize(), cuboidId);
                        if (weights.velocityWeight > 0.) {
                            problem.AddResidualBlock(costFunction, NULL, momentum0, momentum1, impulse, collPoint, parabolaTranslationThis, parabolaTranslationOther);
                        }

                        sprintf(name, "AngVelCostFunctor_cub%d_coll%u", cuboidId, collId);
                        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {momentum0, momentum1, impulse, collPoint, parabolaTranslationThis, parabolaTranslationOther}));
                    }
                } //...for collisions
            } //...for cuboids
        } //...addImpulseTerms()
#if 0
    void addImpulseTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        ceres::FunctorInfosT        & costFunctors,
        FrameIdsT             const & frameIds,
        Weights               const & weights,
        CuboidsT              const & cuboids,
        Consts                const & consts
    )
    {
        if ( cuboids.size() > 2 )
            throw new BundleWithPhysics_ImpulseTerms_TwoCuboidsAssumedException("");

        using ceres::CeresScalar;
        using ceres::CostFunction;

        char name[512];

        CeresScalar *const parabolaShared = indexer.getParabolaSharedParams();
        for ( CuboidId cuboidId = 0; cuboidId != 1; ++cuboidId )
        {
            const Cuboid &cuboid = cuboids.at(cuboidId);
            if ( !cuboid.isMassFinite() )
                throw new BundleWithPhysics_ImpulseTerms_InfiniteMassException("");

            // transitions: 1 transition for 3 frameIds (beg, just before collision, end)
            for ( CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId )
            {
                CeresScalar *const collisionTime = indexer.getCollisionTime(collId);
                CeresScalar *const parabolaFree0 = indexer.getParabolaFreeParams(cuboidId, getPartIdBefore(collId));
                CeresScalar *const parabolaFree1 = indexer.getParabolaFreeParams(cuboidId, getPartIdAfter(collId));

                // velocity
                CeresScalar *const mass    = indexer.getMass(cuboidId);
                CeresScalar *const impulse = indexer.getImpulse(collId);
                {
                    CostFunction *const costFunction = LinVelCostFunctor::Create(weights.velocityWeight, consts.k_fps, frameIds.front(), cuboidId);
                    problem.AddResidualBlock(costFunction, NULL, parabolaShared, parabolaFree0, parabolaFree1, collisionTime, impulse, mass);

                    sprintf(name, "VelocityCostFunctor_cub%d_coll%u", cuboidId, collId);
                    costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {parabolaShared, parabolaFree0, parabolaFree1, collisionTime, impulse, mass}));
                }

                // angVel exchange
                const CuboidId other = getOther(cuboidId, participants);
                std::cout << "other of cuboid" << cuboidId << ", is cuboid" << other << std::endl;
                CeresScalar *const parabolaTranslationThis  = indexer.getParabolaTranslation(cuboidId, collId);
                CeresScalar *const parabolaTranslationOther = indexer.getParabolaTranslation(other, collId);
                //CeresScalar *const poseAddress              = indexer.getCollisionPose(cuboidId, collId);
                CeresScalar *const momentum0                = indexer.getMomentum(cuboidId, getPartIdBefore(collId));
                CeresScalar *const momentum1                = indexer.getMomentum(cuboidId, getPartIdAfter(collId));
                CeresScalar *const collPoint                = indexer.getCollisionPoint(collId);
                {
                    CostFunction *const costFunction = AngVelCostFunctor::Create(weights.velocityWeight, cuboid.getSize(), cuboidId);
                    problem.AddResidualBlock(costFunction, NULL, momentum0, momentum1, impulse, collPoint, parabolaTranslationThis, parabolaTranslationOther, collisionTime);

                    sprintf(name, "AngVelCostFunctor_cub%d_coll%u", cuboidId, collId);
                    costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {momentum0, momentum1, impulse, collPoint, parabolaTranslationThis, parabolaTranslationOther, collisionTime}));
                }
            } //...for collisions
        } //...for cuboids
    } //...addImpulseTerms()
#endif

        DEFINE_EXCEPTION(BundleWithPhysics_FixCollPointNotInUse)
        DEFINE_EXCEPTION(BundleWithPhysics_NoInputCollPoint)

        void initCollPoint(
            ceres::Problem                &      problem,
            PhysIndexer                   &      indexer,
            const Weights                 &      weights,
            const std::vector<CuboidId>   &      participants,
            const Consts                  &      consts,
            const BundleWithPhysicsResult *const initial
        ) {
            using ceres::CeresScalar;
            using ceres::CeresVector3;
            using ceres::MapCeresVector3;
            using ceres::MapConstCeresVector3;
            //typedef Cuboid::Scalar      Scalar;
            //typedef Cuboid::Vector3     Vector3;

            for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
                CeresScalar *const collPoint = indexer.getCollisionPoint(collId);

                if ( weights.initFlags & bundle_physics::Weights::INIT_FLAGS::USE_INPUT_COLLPOINTS ) {
                    if ( !initial )
                        throw new BundleWithPhysics_NoInputCollPointException("");

                    (MapCeresVector3(collPoint)) = initial->collisions.at(collId).relCollPoint.cast<CeresScalar>(); //-0.0575281   0.0592389 -0.00965203
                    std::cout << "inited collPoint to " << MapCeresVector3(collPoint).transpose() << std::endl;
                } else {
                    const CuboidId  objA = getParticipant(0, collId, participants);
                    const CuboidId  objB = getParticipant(1, collId, participants);
                    MapCeresVector3 parabolaTranslationA{indexer.getParabolaTranslation(objA, collId)};
                    MapCeresVector3 parabolaTranslationB{indexer.getParabolaTranslation(objB, collId)};

                    (MapCeresVector3(collPoint)) = (parabolaTranslationB - parabolaTranslationA) / 2.;
                    (MapCeresVector3(indexer.getImpulse(collId))) = (parabolaTranslationB - parabolaTranslationA).normalized(); // added 2016 05 02

                    std::cout << "inited collPoint to " << MapCeresVector3(collPoint).transpose()
                              << ", from tA: " << parabolaTranslationA.transpose()
                              << ", and tB: " << parabolaTranslationB.transpose()
                              << std::endl;
#if 0
                    // debug
                    {
                        Soup::vis::Visualizer<Scalar> v2("initCollPoint");
                        v2.addSphere(parabolaTranslationA.cast<Scalar>(), 0.01, Vector3(1., 0., 0.), "trA");
                        v2.addSphere(parabolaTranslationB.cast<Scalar>(), 0.01, Vector3(0., 1., 0.), "trB");
                        v2.addLine((parabolaTranslationA + MapConstCeresVector3(collPoint)).cast<Scalar>(), parabolaTranslationA.cast<Scalar>(), Vector3::Zero(), "cp");

                        v2.spin();
                    }
#endif
                }

                if ( weights.fixFlags & Weights::FIX_FLAGS::FIX_COLLPOINT )
                    throw new BundleWithPhysics_FixCollPointNotInUseException("");

                problem.SetParameterBlockConstant(collPoint);
            } //...for collisions
        } //...initCollPoint
    } //...ns bundle_physics
} //...ns tracking