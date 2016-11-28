//
// Created by bontius on 24/02/16.
//

#ifndef TRACKVIDEO_COMMON_SAVEPLY_H
#define TRACKVIDEO_COMMON_SAVEPLY_H

#include "tracking/common/eigen.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/typedefs.h"
#include <map>
#include <string>

namespace tracking {
  namespace io {
    template<typename _Tracks3dT> int
    savePly(std::string const& path, _Tracks3dT const& tracks3d, std::map<TrackId, LinId>* const linIdsArg = nullptr,
            std::map<TrackId, Eigen::Vector3f> const* const trackColors = nullptr);
  } //...ns io
} //..ns tracking

#endif //TRACKVIDEO_COMMON_SAVEPLY_H
