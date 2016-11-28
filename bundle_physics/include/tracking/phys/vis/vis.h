//
// Created by bontius on 13/12/15.
//

#ifndef TRACKVIDEO_PHYS_VIS_H
#define TRACKVIDEO_PHYS_VIS_H

#include "tracking/phys/typedefsGeometry.h"
#include "tracking/common/typedefs.h"

namespace tracking {
  namespace bundle_physics {

    class Cuboid;

    template<typename _VisT>
    void drawCuboidStates2(const Cuboid &cuboid, _VisT &vis, const Scalar scale, const Vector3 &color, const int cuboidId, const FrameIdsT *const frameIds, bool posOnly);

    template<typename _VisT>
    void drawCuboidStates(const Cuboid &cuboid, _VisT &vis, const Scalar scale, const Vector3 &color, const int cuboidId, const FrameId currFrame, const FrameIdsT *const frameIds = nullptr);
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_VIS_H
