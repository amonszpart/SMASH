//
// Created by bontius on 04/01/16.
//

#ifndef TRACKVIDEO_PHYS_CONSTS_H
#define TRACKVIDEO_PHYS_CONSTS_H

#include "ceres/ceresUtil.h"
#include "tracking/phys/typedefs.h"
#include "tracking/common/typedefs.h"

namespace tracking {
namespace bundle_physics {

struct Consts {
    Consts(ceres::CeresScalar fps, const FrameIdsT &frameIds)
        : k_dt(1. / fps),
          k_fps(fps),
          substeps(3),
          firstPartId(0),
          lastPartId(frameIds.size() - 2),
          firstCollId(0),
          lastCollId(lastPartId - 1)
    {}

    const ceres::CeresScalar k_dt;
    const ceres::CeresScalar k_fps;
    const int                substeps;
    const PartId             firstPartId;
    const PartId             lastPartId;
    const CollId             firstCollId;
    const CollId             lastCollId;
};

} //...ns bundle_physics
} //...ns tracking
#endif //TRACKVIDEO_PHYS_CONSTS_H
