//
// Created by bontius on 06/01/16.
//

#ifndef TRACKVIDEO_PHYS_POSETERMS_H
#define TRACKVIDEO_PHYS_POSETERMS_H

#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/util/exception.h"
#include "ceres/functorInfo.h"
#include <vector>

namespace ceres { class LossFunctionWrapper; class Problem;}
namespace tracking {
  namespace bundle_physics {
    class Weights;
    class Consts;
    class PhysIndexer;

    DEFINE_EXCEPTION( BundleWithPhysics_PoseTerms_InfiniteMass )
    void addPoseTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        ceres::FunctorInfosT        & costFunctors,
        const FrameIdsT             & frameIds,
        const Weights               & weights,
        const CuboidsT              & cuboids,
        //const std::vector<CuboidId> & participants,
        const Consts                & consts,
        ceres::LossFunctionWrapper  * const poseLoss
    );

  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_POSETERMS_H
