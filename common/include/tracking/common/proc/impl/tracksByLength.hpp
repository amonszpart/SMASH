//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_TRACKSBYLENGTH_HPP
#define TRACKVIDEO_TRACKSBYLENGTH_HPP

#include "tracking/common/proc/tracksByLength.h"
#include <iostream>

namespace tracking {

  template<typename _TrackT>
  GroupsBySize sortBySize(GroupedTracks<_TrackT> const& tracks) {
      GroupsBySize bySize;
      for (auto const& gidAndLinIndices : tracks.getGroups()) {
          bySize.insert(std::make_pair(gidAndLinIndices.second.size(), gidAndLinIndices.first));
      }
      return bySize;
  } //...sortBySize()

} //...ns tracking

#endif //TRACKVIDEO_TRACKSBYLENGTH_HPP
