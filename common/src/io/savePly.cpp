//
// Created by bontius on 15/03/16.
//

#include "tracking/common/io/impl/savePly.hpp"
#include "tracking/common/trackPoint2d.h"
#include "tracking/common/trackPoint3d.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/util/colors.h"
#include <fstream>

namespace tracking {
  namespace io {
    template
    int savePly(std::string const& path, GroupedTracks3d const& tracks3d, std::map<TrackId, LinId>* const linIdsArg,
                std::map<TrackId, Eigen::Vector3f> const* const trackColors);
  } //...ns io
} //..ns tracking
