//
// Created by bontius on 25/05/16.
//

#ifndef TRACKVIDEO_TRACKPOINTFWDECL_H
#define TRACKVIDEO_TRACKPOINTFWDECL_H

#include "soup/geometryTypedefs.h"

namespace tracking {
  class TrackPoint2D;
  template <typename _Scalar> class TrackPoint3DTemplate;
  typedef TrackPoint3DTemplate<Soup::geometry::Scalar> TrackPoint3D;
}

#endif //TRACKVIDEO_TRACKPOINTFWDECL_H
