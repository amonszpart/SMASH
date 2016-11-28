#ifndef TRACKING_COMMON_RGBIO_H
#define TRACKING_COMMON_RGBIO_H

#include "tracking/common/typedefs.h"
#include <vector>

namespace tracking {
    int  readImages(LinRgbsT& rgbs, int argc, const char* argv[], const FrameIdsT& frameIds);
    int  readImages(LinRgbsT& rgbs, std::string imgPattern, const FrameIdsT& frameIds);
    void writeRgb  (cv::Mat const& img, std::string const path);
} //...ns tracking

#endif // TRACKING_COMMON_RGBIO_H
