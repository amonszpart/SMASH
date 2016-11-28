//
// Created by bontius on 31/01/16.
//

#include "tracking/common/trackPoint3d.h"
#include "tracking/common/trackPoint2d.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/proc/impl/tracksByLength.hpp"

namespace tracking {
  template GroupsBySize sortBySize(GroupedTracks<Track2D> const& tracks);
  template GroupsBySize sortBySize(GroupedTracks<Track3D> const& tracks);
} //...ns tracking