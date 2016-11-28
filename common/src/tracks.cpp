#include "tracking/common/trackPoint3d.h"
#include "tracking/common/trackPoint2d.h"
#include "tracking/common/track.h"
#include "tracking/common/impl/tracks.hpp"

template class tracking::TracksTemplate<tracking::Track2D>;
template class tracking::TracksTemplate<tracking::Track3D>;
