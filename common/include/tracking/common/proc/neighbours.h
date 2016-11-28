//
// Created by bontius on 06/04/16.
//

#ifndef TRACKVIDEO_COMMON_NEIGHBOURS_H
#define TRACKVIDEO_COMMON_NEIGHBOURS_H

#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/typedefs.h"
#include <set>

namespace tracking {
/** \brief One to many neighbourhood relations between tracks (based on one time frame). */
typedef std::map<TrackId, std::set<TrackId> > NeighboursT;

namespace common {

/** \brief Estimate the neighbourlist of tracks based on their eucledian distance in space-time (2+1d). */
template <typename _TrackT>
NeighboursT buildNeighbourhood(GroupedTracks<_TrackT> const& tracks2,
                               std::set<FrameId> const& allFrameIds,
                               int const K = 5,
                               std::function<bool(TrackId const&)>* useTrack = nullptr);
} //...ns common
} //...ns tracking

#endif //TRACKVIDEO_COMMON_NEIGHBOURS_H
