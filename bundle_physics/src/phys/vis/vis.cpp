//
// Created by bontius on 28/03/16.
//

#include "tracking/phys/vis/vis.h"
#include "tracking/phys/vis/impl/vis.hpp"
#include "tracking/vis/visualizer.h"


namespace tracking {
  namespace bundle_physics {

    template
    void drawCuboidStates2(const Cuboid &cuboid, Soup::vis::Visualizer<Scalar> &vis, const Scalar scale, const Vector3 &color, const int cuboidId, const FrameIdsT *const frameIds, bool const posOnly);

    template
    void drawCuboidStates(const Cuboid &cuboid, Soup::vis::Visualizer<Scalar> &vis, const Scalar scale, const Vector3 &color, const int cuboidId, const FrameId currFrame, const FrameIdsT *const frameIds);

  } //...ns bundle_physics
} //...ns tracking
