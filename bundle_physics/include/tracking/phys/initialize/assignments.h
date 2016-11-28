//
// Created by bontius on 29/03/16.
//

#ifndef TRACKVIDEO_PHYS_ASSIGNMENTS_H
#define TRACKVIDEO_PHYS_ASSIGNMENTS_H

#include "tracking/phys/typedefs.h"
#include "tracking/common/assignments.h"
#include <cstddef>

namespace tracking {
  namespace bundle_physics {
    using TracksToCuboidsT = TracksToTarget<CuboidId,-1>;
  } //...ns bundle_phsyics
} //...ns tracking
#endif //TRACKVIDEO_PHYS_ASSIGNMENTS_H
