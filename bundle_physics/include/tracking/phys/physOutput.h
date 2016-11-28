//
// Created by bontius on 29/03/16.
//

#ifndef TRACKVIDEO_PHYS_PHYSOUTPUT_H
#define TRACKVIDEO_PHYS_PHYSOUTPUT_H

#include "tracking/phys/initialize/assignments.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/groupedTracksFwDecl.h"

namespace tracking {
namespace bundle_physics {
class BundleWithPhysicsResult;
class PhysIndexer;
class Consts;
class Weights;

void output(BundleWithPhysicsResult& out,
            CuboidsT const& cuboids,
            GroupedTracks2d const& tracks2d,
            FrameIdsT const& frameIds,
            PhysIndexer const& indexer,
            Consts const& consts,
            bool const doVis = true,
            TracksToCuboidsT const* const assignments = nullptr);
} //...ns bundle_phsyics
} //...ns tracking


#endif //TRACKVIDEO_PHYS_PHYSOUTPUT_H
