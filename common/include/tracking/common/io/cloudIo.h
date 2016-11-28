//
// Created by bontius on 13/06/16.
//

#ifndef TRACKVIDEO_COMMON_CLOUDIO_H
#define TRACKVIDEO_COMMON_CLOUDIO_H

#include "tracking/common/clouds/typedefsCloud.h"
#include "tracking/common/typedefs.h"

namespace tracking {
/** \brief Reads images listed in depthFrameIds (no interval). Parses '--cloud-pattern' */
int readPlys(ConstCloudsT& clouds, int argc, const char** argv, FrameIdsT const& depthFrameIds,
             std::string const& defaultCloudPattern = "--cloud-pattern");

/** \brief Reads images listed in depthFrameIds (no interval). */
int readPlys(ConstCloudsT& clouds, std::string cloudPattern, FrameIdsT const& depthFrameIds);
} //...ns tracking



#endif //TRACKVIDEO_CLOUDIO_H
