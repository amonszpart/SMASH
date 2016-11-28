//
// Created by bontius on 31/03/16.
//

#ifndef TRACKVIDEO_SFMIMPULSETERMS_H
#define TRACKVIDEO_SFMIMPULSETERMS_H

#include "tracking/phys/physFunctorInfoFwDecl.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/util/exception.h"
#include "tracking/common/typedefs.h"
#include "ceres/functorInfo.h"
#include "ceres/ceresUtil.h"

namespace ceres { class Problem; class LossFunctionWrapper; }

namespace tracking {
  class Mapper;
  namespace bundle_physics {
    class Weights;
    class Consts;
    class PhysIndexer;
    class BundleWithPhysicsResult;

    DEFINE_EXCEPTION(BundleWithPhysics_ImpulseTerms_InfiniteMass)
    DEFINE_EXCEPTION(BundleWithPhysics_ImpulseTerms_TwoCuboidsAssumed)

    void addImpulseTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        PhysFunctorInfosT           & costFunctors,
        const FrameIdsT             & frameIds,
        const Weights               & weights,
        const CuboidsT              & cuboids,
        const std::vector<CuboidId> & participants,
        const Consts                & consts
    );
  } //...ns bundle_physics
} //...ns tracking


#endif //TRACKVIDEO_SFMIMPULSETERMS_H
