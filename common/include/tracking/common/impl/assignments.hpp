//
// Created by bontius on 28/03/16.
//

#ifndef TRACKVIDEO_COMMON_ASSIGNMENTS_HPP
#define TRACKVIDEO_COMMON_ASSIGNMENTS_HPP

#include "tracking/common/assignments.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/track.h"
#include "tracking/common/typedefs.h"

namespace tracking {

  /** \param[out] assignments Output assignments, track to joint assignments will be appended to this list. */
  template<typename _TargetId, size_t _InvalidId>
  void TracksToTarget<_TargetId, _InvalidId>::assignGroupToTarget(TracksToTarget& assignments, GroupId const groupId,
                                                                  _TargetId const targetId,
                                                                  GroupedTracks2d const& tracks2d) {
      std::vector<Track2D const*> tracks = tracks2d.getTracksInGroup(groupId);
      for (Track2D const* const& track : tracks) {
          assignments[track->getLabel()] = targetId;
      }
  }

  template<typename _TargetId, size_t _InvalidId>
  typename TracksToTarget<_TargetId, _InvalidId>::InverseT
  TracksToTarget<_TargetId, _InvalidId>::invert(TracksToTarget <_TargetId, _InvalidId> const& assignments) {
      InverseT tracksByJoint;
      for (auto const& trackAndJoint : assignments) {
          tracksByJoint.insert({trackAndJoint.second, trackAndJoint.first});
      }
      return tracksByJoint;
  } //...TracksToJointsT::invert()
} //...ns tracking

#endif //TRACKVIDEO_COMMON_ASSIGNMENTS_HPP
