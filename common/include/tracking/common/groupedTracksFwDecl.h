//
// Created by bontius on 16/02/16.
//

#ifndef TRACKVIDEO_COMMON_GROUPEDTRACKSFWDECL_H
#define TRACKVIDEO_COMMON_GROUPEDTRACKSFWDECL_H

#include "tracking/common/trackFwDecl.h"

namespace tracking {
  template <typename _TrackT> class GroupedTracks;

  typedef GroupedTracks<Track3D> GroupedTracks3d;
  typedef GroupedTracks<Track2D> GroupedTracks2d;

} //...ns tracking

#endif //TRACKVIDEO_COMMON_GROUPEDTRACKSFWDECL_H
