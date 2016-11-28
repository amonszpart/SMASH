#ifndef TV_TEST5POINTS_H
#define TV_TEST5POINTS_H

#include "tracking/common/tracks.h"
#include "tracking/common/mapper.h"

namespace tracking
{
    int test5Points( const Tracks2D& tracks2d, const FrameIdsT& frameIds, const Mapper& mapper );
} //...ns tracking

#endif // TV_TEST5POINTS_H
