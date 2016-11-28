//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_SEGMENTED_H
#define TRACKVIDEO_SEGMENTED_H

#include "tracking/common/groupedTracksFwDecl.h"
#include <string>

namespace tracking {
  namespace io {

    /** \addtogroup common
     * @{ */
    /** \addtogroup io
     * @{ */

    template <typename _TrackT>
    int readSegmentedTracks( std::string const& path, GroupedTracks<_TrackT> &tracks );
    int writeSegmentedTracks( GroupedTracks2d const& tracks, std::string const& path );

    /** @} (io) */
    /** @} (common) */

  } //...ns io
} //..ns tracking

#endif //TRACKVIDEO_SEGMENTED_H
