//
// Created by bontius on 28/01/16.
//

#include "tracking/common/trackPoint3d.h"
#include "tracking/common/trackPoint2d.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/impl/groupedTracks.hpp"

template class tracking::GroupedTracks<tracking::Track2D>;
template class tracking::GroupedTracks<tracking::Track3D>;

//deliberately causing overflow
template<int N>
struct print_size_as_warning { char operator()() { return N + 256; }};
void test() {
    print_size_as_warning<sizeof(tracking::Track3D)>();
}