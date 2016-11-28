//
// Created by bontius on 31/03/16.
//

#include "tracking/phys/energyTerms/sfmImpulseTerm.h"
#include "tracking/phys/energyTerms/impl/impulseTerm.hpp"
#include "tracking/phys/physFunctorInfo.h"
#include "tracking/phys/partUtil.h"
#include "tracking/phys/weights.h"
#include "tracking/phys/consts.h"
#include "tracking/annot/cuboid.h"
#include "ceres/problem.h"
#include "tracking/unique_ptr.h"
#include <vector>


namespace tracking {
  namespace bundle_physics {
    void addImpulseTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        PhysFunctorInfosT           & costFunctors,
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
        for ( CuboidId cuboidId = 0; cuboidId != CuboidId(cuboids.size()); ++cuboidId )
        {
            const Cuboid &cuboid = cuboids.at(cuboidId);
            if ( !cuboid.isMassFinite() )
                throw new BundleWithPhysics_ImpulseTerms_InfiniteMassException("");

            // transitions: 1 transition for 3 frameIds (beg, just before collision, end)
            for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId)
            {
                //CeresScalar *const collisionTime = indexer.getCollisionTime(collId);
                CeresScalar *const parabolaFree0 = indexer.getParabolaFreeParams(cuboidId, getPartIdBefore(collId));
                CeresScalar *const parabolaFree1 = indexer.getParabolaFreeParams(cuboidId, getPartIdAfter(collId));

                // velocity
                CeresScalar *const mass    = indexer.getMass(cuboidId);
                CeresScalar *const impulse = indexer.getImpulse(collId);
                {
                    CostFunction *const costFunction = LinVelCostFunctor::Create(weights.velocityWeight, consts.k_fps, frameIds.front(),cuboidId);
                    ceres::ResidualBlockId residualBlockId = problem.AddResidualBlock(costFunction, NULL, parabolaRotShared, parabolaA, parabolaFree0, parabolaFree1, /*collisionTime, */impulse, mass);

                    sprintf(name, "VelocityCostFunctor_cub%d_coll%u", cuboidId, collId);
                    costFunctors.emplace_back(std::make_shared<PhysFunctorInfo>(
                        name, costFunction, std::vector<CeresScalar*>{parabolaRotShared,parabolaA,parabolaFree0,parabolaFree1,impulse,mass},residualBlockId));
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
                    ceres::ResidualBlockId residualBlockId = problem.AddResidualBlock(costFunction, NULL, momentum0, momentum1, impulse, collPoint, parabolaTranslationThis, parabolaTranslationOther/*, collisionTime*/);

                    sprintf(name, "AngVelCostFunctor_cub%d_coll%u", cuboidId, collId);
                    costFunctors.emplace_back(std::make_shared<PhysFunctorInfo>(
                        name, costFunction, std::vector<CeresScalar*>{momentum0, momentum1, impulse, collPoint, parabolaTranslationThis, parabolaTranslationOther},residualBlockId));
                }
            } //...for collisions
        } //...for cuboids
    } //...addImpulseTerms()
  }
}