//
// Created by bontius on 10/01/16.
//

#ifndef TRACKVIDEO_PHYS_INFMASS_H
#define TRACKVIDEO_PHYS_INFMASS_H

#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/util/exception.h"

namespace tracking {
class Mapper;
namespace bundle_physics {
class Weights;
class BundleWithPhysicsResult;

DEFINE_EXCEPTION( BundleWithPhysics_InfMass_FiniteCuboidsFirstAssumed )
int infMass(
    BundleWithPhysicsResult   &out,
    const CuboidsT                  &cuboids,
    const GroupedTracks2d           &tracks2d,
    const LinRgbsT                  &rgbs,
    const FrameIdsT                 &frameIds,
    const Mapper                    &mapper,
    const Weights                   &weights,
    const BundleWithPhysicsResult   *const initial,
    std::string const& showFlags
);

} //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_INFMASS_H
