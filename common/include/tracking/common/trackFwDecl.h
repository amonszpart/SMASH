//
// Created by bontius on 16/02/16.
//

#ifndef TRACKVIDEO_COMMON_TRACKFWDECL_H
#define TRACKVIDEO_COMMON_TRACKFWDECL_H

#include "tracking/common/trackPointFwDecl.h"
#include <memory>

namespace tracking {

template <typename _PointT> class TrackTemplate;

typedef TrackTemplate  <TrackPoint2D> Track2D;
using Track2d = Track2D;
typedef TrackTemplate  <TrackPoint3D> Track3D;
using Track3d = Track3D;
typedef std::shared_ptr<Track3D>      Track3DPtr;
} //...ns tracking

#endif //TRACKVIDEO_COMMON_TRACKFWDECL_H
