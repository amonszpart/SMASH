//
// Created by bontius on 21/02/16.
//

#ifndef TRACKVIDEO_COMMON_ASSIGNMENTSIO_H
#define TRACKVIDEO_COMMON_ASSIGNMENTSIO_H

#include "tracking/common/assignments.h"
#include "tracking/common/tracksFwDecl.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/util/exception.h"

namespace tracking {
  namespace io {
    /** \brief Read track-to-joint assignments from file. */
    template<typename _TargetId, size_t _InvalidId>
    int readAssignments(TracksToTarget<_TargetId, _InvalidId>& assignments, std::string const& assignmentsPath,
                        GroupedTracks2d const& tracks2d, std::string const& tagName = "jointId");

    /** \brief Write track-to-joint assignments to file. */
    template<typename _TargetId, size_t _InvalidId>
    int writeAssignments(TracksToTarget<_TargetId, _InvalidId> const& assignments, std::string const& assignmentsPath,
                         TrackIds2LinIds const* const trackIds2LinIds = nullptr,
                         std::string const& targetTag = "jointId");
  } //...ns io
} //...ns tracking

#endif //TRACKVIDEO_COMMON_ASSIGNMENTSIO_H
