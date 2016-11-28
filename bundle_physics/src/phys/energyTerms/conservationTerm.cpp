//
// Created by bontius on 06/01/16.
//

#include "tracking/phys/energyTerms/conservationTerm.h"
#include "tracking/phys/energyTerms/impl/conservationTerm.hpp"
#include "tracking/phys/physFunctorInfo.h"

#include "tracking/annot/cuboid.h"
#include "tracking/phys/weights.h"
#include "tracking/phys/consts.h"
#include "tracking/phys/partUtil.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/unique_ptr.h"
#include "ceres/problem.h"

namespace tracking {
  namespace bundle_physics {

    void addConservationTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        ceres::FunctorInfosT        & costFunctors,
        const FrameIdsT             & frameIds,
        const Weights               & weights,
        const CuboidsT              & cuboids,
        const std::vector<CuboidId> & participants,
        const Consts                & consts
    )
    {
        using ceres::CeresScalar;
        using ceres::CostFunction;
        char name[255];

        for ( CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
            const CuboidId  objA         = getParticipant ( 0, collId, participants);
            const CuboidId  objB         = getParticipant ( 1, collId, participants);
            if ( !cuboids.at(objA).isMassFinite() )
                throw BundleWithPhysics_AddConservationTerms_InfiniteMassException("");
            if ( !cuboids.at(objB).isMassFinite() )
                throw BundleWithPhysics_AddConservationTerms_InfiniteMassException("");

            const int       partIdBefore = getPartIdBefore( collId );
            const int       partIdAfter  = getPartIdAfter ( collId );

            CeresScalar *const massA                = indexer.getMass(objA);
            CeresScalar *const massB                = indexer.getMass(objB);
            //CeresScalar *const parabolaShared       = indexer.getParabolaSharedParams();
            CeresScalar *const parabolaRotShared    = indexer.getParabolaRotationShared();
            CeresScalar *const parabolaA            = indexer.getParabolaSquaredParam();
            CeresScalar *const parabolaTranslationA = indexer.getParabolaTranslation(objA, collId);
            CeresScalar *const parabolaFreeA0       = indexer.getParabolaFreeParams(objA, partIdBefore);
            CeresScalar *const parabolaFreeA1       = indexer.getParabolaFreeParams(objA, partIdAfter);
            CeresScalar *const parabolaTranslationB = indexer.getParabolaTranslation(objB, collId);
            CeresScalar *const parabolaFreeB0       = indexer.getParabolaFreeParams(objB, partIdBefore);
            CeresScalar *const parabolaFreeB1       = indexer.getParabolaFreeParams(objB, partIdAfter);
            CeresScalar *const collisionTime        = indexer.getCollisionTime(collId);
            // conservation of linear momentum
            {
                CostFunction *costFunction = ConservationOfLinearMomentum::Create(weights.conservationWeight, consts.k_fps, consts.k_dt, CeresScalar(frameIds.front()));
                problem.AddResidualBlock(costFunction, NULL, massA, massB, parabolaRotShared, parabolaA, parabolaFreeA0, parabolaFreeA1, parabolaFreeB0, parabolaFreeB1/*, collisionTime*/);
                std::cout << "conservation weight: " << weights.conservationWeight << std::endl;
                sprintf(name, "ConservationOfLinearMomentum_coll%u", collId);
                costFunctors.emplace_back(
                    ceres::FunctorInfo(name, costFunction, {massA, massB, parabolaRotShared, parabolaA, parabolaFreeA0, parabolaFreeA1, parabolaFreeB0, parabolaFreeB1/*, collisionTime*/}));
            }

            CeresScalar *const momentumA0 = indexer.getMomentum(objA, partIdBefore);
            CeresScalar *const momentumA1 = indexer.getMomentum(objA, partIdAfter);
            CeresScalar *const momentumB0 = indexer.getMomentum(objB, partIdBefore);
            CeresScalar *const momentumB1 = indexer.getMomentum(objB, partIdAfter);

            {
                CostFunction *costFunction = ConservationOfAngularMomentum::Create(weights.conservationWeight, consts.k_fps, consts.k_dt, CeresScalar(frameIds.front()));
                std::vector<CeresScalar *> unknowns = {
                    massA, massB, parabolaRotShared, parabolaA,
                    parabolaTranslationA, parabolaFreeA0, parabolaFreeA1,
                    parabolaTranslationB, parabolaFreeB0, parabolaFreeB1,
                    collisionTime, momentumA0, momentumA1, momentumB0, momentumB1
                };
                #warning normalize all terms
                // todo: normalize term!
                problem.AddResidualBlock(costFunction, NULL, unknowns);

                sprintf(name, "ConservationOfAngularMomentum_coll%u", collId);
                costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, unknowns));
            } // angular momentum
        } //...for collisions
    } //...addConservationTerms

    void addConservationTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        ceres::FunctorInfosT        & costFunctors,
        const FrameIdsT             & frameIds,
        const Weights               & weights,
        const Cuboid                & /*cuboid*/, // assumed to have id 0 in indexer
        const Consts                & consts)
    {
#if 1
        using ceres::CeresScalar;
        using ceres::CostFunction;
        char name[255];

        for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
            const CuboidId  objA         = 0;
            const int       partIdBefore = getPartIdBefore( collId );
            const int       partIdAfter  = getPartIdAfter ( collId );

            CeresScalar *const massA                = indexer.getMass(objA);
            //CeresScalar *const massB                = indexer.getMass(objB);
            //CeresScalar *const parabolaShared       = indexer.getParabolaSharedParams();
            CeresScalar *const parabolaRotShared    = indexer.getParabolaRotationShared();
            CeresScalar *const parabolaA            = indexer.getParabolaSquaredParam();
            CeresScalar *const parabolaTranslationA = indexer.getParabolaTranslation(objA, collId);
            CeresScalar *const parabolaFreeA0       = indexer.getParabolaFreeParams(objA, partIdBefore);
            CeresScalar *const parabolaFreeA1       = indexer.getParabolaFreeParams(objA, partIdAfter);
            //CeresScalar *const parabolaTranslationB = indexer.getParabolaTranslation(objB, collId);
            //CeresScalar *const parabolaFreeB0       = indexer.getParabolaFreeParams(objB, partIdBefore);
            //CeresScalar *const parabolaFreeB1       = indexer.getParabolaFreeParams(objB, partIdAfter);
            CeresScalar *const collisionTime        = indexer.getCollisionTime(collId);
            // conservation of linear momentum
            {
                CostFunction *costFunction = ConservationOfLinearMomentumInfMass::Create(weights.conservationWeight, consts.k_fps, consts.k_dt, CeresScalar(frameIds.front()));
                problem.AddResidualBlock(costFunction, NULL, massA, parabolaRotShared, parabolaA, parabolaFreeA0, parabolaFreeA1/*, collisionTime*/);

                sprintf(name, "ConservationOfLinearMomentumInfMass_coll%u", collId);
                costFunctors.emplace_back(
                    ceres::FunctorInfo(name, costFunction, {massA, parabolaRotShared, parabolaA, parabolaFreeA0, parabolaFreeA1/*, collisionTime*/}));
            }

            CeresScalar *const momentumA0 = indexer.getMomentum(objA, partIdBefore);
            CeresScalar *const momentumA1 = indexer.getMomentum(objA, partIdAfter);

            {
                CostFunction *costFunction = ConservationOfAngularMomentumInfMass::Create(weights.conservationWeight, consts.k_fps, consts.k_dt, CeresScalar(frameIds.front()));
                std::vector<CeresScalar *> unknowns = {massA, parabolaRotShared, parabolaA,
                                                       parabolaTranslationA, parabolaFreeA0, parabolaFreeA1,
                    //parabolaTranslationB, parabolaFreeB0, parabolaFreeB1,
                                                       collisionTime, momentumA0, momentumA1,
                    //momentumB0, momentumB1
                };
                problem.AddResidualBlock(costFunction, NULL, unknowns);

                sprintf(name, "ConservationOfAngularMomentumInfMass_coll%u", collId);
                costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, unknowns));
            } // angular momentum
        } //...for collisions
#endif
    } //...addConservationTerms

    void sfmAddConservationTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        PhysFunctorInfosT           & costFunctors,
        FrameIdsT             const & frameIds,
        Weights               const & weights,
        CuboidsT              const & cuboids,
        std::vector<CuboidId> const & participants,
        Consts                const & consts
    ){
        using ceres::CeresScalar;
        using ceres::CostFunction;
        char name[255];

        for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
            const CuboidId  objA         = getParticipant (0, collId, participants);
            const CuboidId  objB         = getParticipant (1, collId, participants);
            if ( !cuboids.at(objA).isMassFinite() )
                throw BundleWithPhysics_AddConservationTerms_InfiniteMassException("");
            if ( !cuboids.at(objB).isMassFinite() )
                throw BundleWithPhysics_AddConservationTerms_InfiniteMassException("");

            const int       partIdBefore = getPartIdBefore( collId );
            const int       partIdAfter  = getPartIdAfter ( collId );

            CeresScalar *const massA                = indexer.getMass(objA);
            CeresScalar *const massB                = indexer.getMass(objB);
            //CeresScalar *const parabolaShared       = indexer.getParabolaSharedParams();
            CeresScalar *const parabolaRotShared    = indexer.getParabolaRotationShared();
            CeresScalar *const parabolaA            = indexer.getParabolaSquaredParam();
            CeresScalar *const parabolaTranslationA = indexer.getParabolaTranslation(objA, collId);
            CeresScalar *const parabolaFreeA0       = indexer.getParabolaFreeParams(objA, partIdBefore);
            CeresScalar *const parabolaFreeA1       = indexer.getParabolaFreeParams(objA, partIdAfter);
            CeresScalar *const parabolaTranslationB = indexer.getParabolaTranslation(objB, collId);
            CeresScalar *const parabolaFreeB0       = indexer.getParabolaFreeParams(objB, partIdBefore);
            CeresScalar *const parabolaFreeB1       = indexer.getParabolaFreeParams(objB, partIdAfter);
            CeresScalar *const collisionTime        = indexer.getCollisionTime(collId);
            // conservation of linear momentum
            {
                CostFunction *costFunction = ConservationOfLinearMomentum::Create(weights.conservationWeight, consts.k_fps, consts.k_dt, CeresScalar(frameIds.front()));
                ceres::ResidualBlockId residualBlockId = problem.AddResidualBlock(costFunction, NULL, massA, massB, parabolaRotShared, parabolaA, parabolaFreeA0, parabolaFreeA1, parabolaFreeB0, parabolaFreeB1/*, collisionTime*/);
                std::cout << "conservation weight: " << weights.conservationWeight << std::endl;
                sprintf(name, "ConservationOfLinearMomentum_coll%u", collId);
                costFunctors.emplace_back(std::make_shared<PhysFunctorInfo>(
                    name, costFunction, std::vector<CeresScalar*>{massA, massB, parabolaRotShared, parabolaA, parabolaFreeA0, parabolaFreeA1, parabolaFreeB0, parabolaFreeB1},residualBlockId));
            }

            CeresScalar *const momentumA0 = indexer.getMomentum(objA, partIdBefore);
            CeresScalar *const momentumA1 = indexer.getMomentum(objA, partIdAfter);
            CeresScalar *const momentumB0 = indexer.getMomentum(objB, partIdBefore);
            CeresScalar *const momentumB1 = indexer.getMomentum(objB, partIdAfter);

            {
                CostFunction *costFunction = ConservationOfAngularMomentum::Create(weights.conservationWeight, consts.k_fps, consts.k_dt, CeresScalar(frameIds.front()));
                std::vector<CeresScalar *> unknowns = {
                    massA, massB, parabolaRotShared, parabolaA,
                    parabolaTranslationA, parabolaFreeA0, parabolaFreeA1,
                    parabolaTranslationB, parabolaFreeB0, parabolaFreeB1,
                    collisionTime, momentumA0, momentumA1, momentumB0, momentumB1
                };
                ceres::ResidualBlockId residualBlockId = problem.AddResidualBlock(costFunction, NULL, unknowns);

                sprintf(name, "ConservationOfAngularMomentum_coll%u", collId);
                costFunctors.emplace_back(std::make_shared<PhysFunctorInfo>(name, costFunction,unknowns,residualBlockId));
            } // angular momentum
        } //...for collisions
    } //...addConservationTerms

  } //...ns bundle_physics
} //...ns tracking