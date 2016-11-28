//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_COMMON_TRACKSBYLENGTH_H
#define TRACKVIDEO_COMMON_TRACKSBYLENGTH_H

#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/typedefs.h"
#include <map>

namespace tracking
{

typedef std::multimap<TrackId,GroupId> GroupsBySize;

template <typename _TrackT>
GroupsBySize sortBySize( GroupedTracks<_TrackT> const& tracks );

} //...ns tracking

#endif //TRACKVIDEO_TRACKSBYLENGTH_H
