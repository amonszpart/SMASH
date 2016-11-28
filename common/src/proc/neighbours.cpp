//
// Created by bontius on 06/04/16.
//

#include "tracking/common/proc/impl/neighbours.hpp"
#include "tracking/common/trackPoint2d.h"
#include "tracking/common/trackPoint3d.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/typedefs.h"
#include <map>

namespace tracking {
namespace common {
template NeighboursT
buildNeighbourhood(GroupedTracks2d const& tracks2, std::set<FrameId> const& allFrameIds, int const K,
                   std::function<bool(TrackId const&)>* useTrack);

// warning: code not ready for 3D (dim = ...*2 => dim = ...*3, and copy stuff...)

} //...ns common
} //...ns tracking