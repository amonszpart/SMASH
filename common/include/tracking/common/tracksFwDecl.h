//
// Created by bontius on 01/03/16.
//

#ifndef TRACKVIDEO_COMMON_TRACKSFWDECL_H
#define TRACKVIDEO_COMMON__TRACKSFWDECL_H

#include "tracking/common/trackFwDecl.h"
#include "tracking/common/typedefs.h"
#include <memory>
#include <map>

namespace tracking {

template <typename _TrackT> class TracksTemplate;

typedef TracksTemplate<Track2D>   Tracks2D;
using Tracks2d = Track2D;
typedef TracksTemplate<Track3D>   Tracks3D;
using Tracks3d = Track3D;
typedef std::shared_ptr<Tracks3D> Tracks3DPtr;
typedef std::map<TrackId,LinId>   TrackIds2LinIds;

} //...ns tracking

#endif //TRACKVIDEO_COMMON_TRACKSFWDECL_H
