//
// Created by bontius on 10/01/16.
//

#ifndef TRACKVIDEO_PHYS_TRIPLECOLL_H
#define TRACKVIDEO_PHYS_TRIPLECOLL_H

#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/util/exception.h"

namespace tracking {
class Mapper;
namespace bundle_physics {
class Weights;
class BundleWithPhysicsResult;

int tripleColl(
    BundleWithPhysicsResult      &       out,
    CuboidsT                const&       cuboids,
    GroupedTracks2d         const&       tracks2d,
    LinRgbsT                const&       rgbs,
    FrameIdsT               const&       frameIds,
    Mapper                  const&       mapper,
    Weights                 const&       weights,
    BundleWithPhysicsResult const* const initial            = nullptr,
    std::string             const        showFlags          = "11111"
);

} //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_TRIPLECOLL_H
