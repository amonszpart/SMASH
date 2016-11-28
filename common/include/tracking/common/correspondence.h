#ifndef TRACKVIDEO_COMMON_CORRESPONDENCE_H
#define TRACKVIDEO_COMMON_CORRESPONDENCE_H

#include "tracking/common/trackPoint2d.h"
#include "tracking/common/trackPoint3d.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "soup/geometryTypedefs.h"
#include <map>

namespace tracking {
    struct TrackIdFrameId : public std::pair<TrackId,FrameId> {
            typedef std::pair<TrackId,FrameId> Base;
            inline TrackIdFrameId( const TrackId &trackId, const FrameId& frameId ) : Base( std::make_pair(trackId,frameId) ) {}
            inline const TrackId& getTrackId() const { return this->first; }
            inline const FrameId& getFrameId() const { return this->second; }
    };

    struct CorrPair : public std::pair<TrackIdFrameId, TrackIdFrameId> {
            typedef std::pair< TrackIdFrameId, TrackIdFrameId > Base;
            inline CorrPair( const TrackIdFrameId &source, const TrackIdFrameId& target ) : Base( std::make_pair(source,target) ) {}
            inline const TrackIdFrameId& getSource() const { return this->first; }
            inline const TrackIdFrameId& getTarget() const { return this->second; }
    };
    typedef std::vector< CorrPair  > CorrPairs;
    typedef std::vector< CorrPairs > CorrPairsList;

    class Correspondence {
        public:
            typedef Soup::geometry::Scalar Scalar;
            typedef Soup::geometry::HelperTypes<Scalar>::Transform TransformationT;

            static bool computeTransform( TransformationT& transform, const Tracks3D& tracks, const CorrPairs& corrs );
    }; //...Correspondence()

    typedef std::map< FrameId, Correspondence::TransformationT> TransformsT;
} //...ns tracking

#endif // TRACKVIDEO_COMMON_CORRESPONDENCE_H

