//
// Created by bontius on 05/02/16.
//

#ifndef TRACKVIDEO_DEPTHIO_H
#define TRACKVIDEO_DEPTHIO_H

#include "tracking/common/typedefs.h"
#include <vector>

namespace tracking
{
  int readDepths(tracking::DepthsT& depths, int argc, const char* argv[], const FrameIdsT& frameIds, float const scale = 0.001f);
  int readDepths(tracking::DepthsT& depths, std::string imgPattern, const FrameIdsT& frameIds, float const scale = 0.001f);
} //...ns tracking

#endif //TRACKVIDEO_DEPTHIO_H
