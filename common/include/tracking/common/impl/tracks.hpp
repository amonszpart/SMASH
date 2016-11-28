#ifndef TRACKVIDEO_COMMON_TRACKS_HPP
#define TRACKVIDEO_COMMON_TRACKS_HPP

#include "tracking/common/tracks.h"
#include <algorithm>
#include <iostream>

namespace tracking {

template <typename _TrackT>
inline _TrackT& TracksTemplate<_TrackT>::addTrack(_TrackT &&track) {
    if (_updated)
        _trackIdLinIds[track.getTrackId()] = _tracks.size();
    else if (_trackIdLinIds.find(track.getTrackId()) != _trackIdLinIds.end())
        throw new Tracks_AddTrack_CacheMismatchException();
    _tracks.emplace_back(std::move(track));
    return this->_tracks.back();
}

template <typename _TrackT>
inline _TrackT& TracksTemplate<_TrackT>::addTrack(_TrackT const& track) {
    if (_updated)
        _trackIdLinIds[track.getTrackId()] = _tracks.size();
    else if (_trackIdLinIds.find(track.getTrackId()) != _trackIdLinIds.end())
        throw new Tracks_AddTrack_CacheMismatchException();
    _tracks.push_back(track);
    return this->_tracks.back();
}

template <typename _TrackT>
template <typename _FunctorT>
inline void TracksTemplate<_TrackT>::filterTracks(TracksTemplate<_TrackT> &tracks, const _FunctorT& functor) {
    TracksTemplate<_TrackT> tmp = tracks; // TODO optimize
    tracks.clear();
    for (auto it = tmp.begin(); it != tmp.end(); ++it) {
        if ( functor.operator()( *it ) )
            tracks.addTrack( *it );
    }
}

template <typename _TrackT>
inline _TrackT& TracksTemplate<_TrackT>::getTrackByLabel(LabelT const label) {
    return const_cast<_TrackT&>(const_cast<TracksTemplate const*>(this)->getTrackByLabel(label));
    #if 0
    if (!_updated)
        _updateCache();
    auto iter = _trackIdLinIds.find(label);
    if (iter == _trackIdLinIds.end())
        throw new Tracks_GetTrackByLabel_NoSuchLabelException( "no such label" );
    else if (iter->second != label)
        throw new Tracks_GetTrackByLabel_CacheMismatchException("very bad");
    else
        return _tracks.at(iter->second);
    #endif
    //auto it = std::find_if( this->_tracks.begin(), this->_tracks.end(), [&label]( const _TrackT& track ) { return label == track.getLabel(); } );
    //if ( it != this->_tracks.end() )    return *it;
    //else                                throw new Tracks_GetTrackByLabel_NoSuchLabelException( "no such label" );
}

template <typename _TrackT>
inline const _TrackT& TracksTemplate<_TrackT>::getTrackByLabel(LabelT const label) const {
    if (!_updated)
        _updateCache();

    auto iter = _trackIdLinIds.find(label);
    if (iter == _trackIdLinIds.end()) {
        if (std::find_if(this->_tracks.begin(), this->_tracks.end(), [&label](_TrackT const& track) {
            return label == track.getLabel(); }) != _tracks.end()) {
            std::cerr << "[" << __func__ << "] yes it does..." << std::endl;
        }
        throw new Tracks_GetTrackByLabel_NoSuchLabelException("no such label: " + std::to_string(label));
    }
    else if (_tracks.at(iter->second).getTrackId() != static_cast<TrackId>(label))
        throw new Tracks_GetTrackByLabel_CacheMismatchException(
            "very bad: " + std::to_string(iter->second) + " vs " + std::to_string(label) );
    else
        return _tracks.at(iter->second);

//    auto it = std::find_if(this->_tracks.begin(), this->_tracks.end(), [&label](_TrackT const& track) {
//        return label == track.getLabel(); });
//    if (it != this->_tracks.end())
//        return *it;
//    else
//        throw new Tracks_GetTrackByLabel_NoSuchLabelException( "No such label: " + std::to_string(label) );
}

template <typename _TrackT> inline typename TracksTemplate<_TrackT>::Base::const_iterator
TracksTemplate<_TrackT>::getTrackByTrackId(TrackId const trackId) const {
    if (!_updated)
        _updateCache();
    auto iter = _trackIdLinIds.find(trackId);
    if (iter == _trackIdLinIds.end())
        return _tracks.end();
    else {
        auto iter2 = _tracks.begin() + iter->second;
        if (iter2->getTrackId() != trackId)
            throw new Tracks_GetTrackByTrackId_CacheMismatchException("", __FILE__, __LINE__);
        return iter2;
    }
    //return std::find_if(this->_tracks.begin(), this->_tracks.end(), [&trackId](_TrackT const& track) { return trackId == static_cast<TrackId>(track.getLabel()); });
}

template <typename _TrackT> inline LinId
TracksTemplate<_TrackT>::getLinIdByLabel(LabelT const label) const
{ return getLinIdByTrackId(static_cast<TrackId>(label)); }

template <typename _TrackT> inline LinId
TracksTemplate<_TrackT>::getLinIdByTrackId(TrackId const trackId) const {
    if (!_updated)
        _updateCache();

    auto iter = _trackIdLinIds.find(trackId);
    if (iter == _trackIdLinIds.end()) {
        throw new Tracks_GetTrackByLabel_NoSuchLabelException("No such label " + std::to_string(trackId),
                                                              __FILE__, __LINE__);
        return LinId(-1);
    } else {
        return iter->second;
    }
} //...getLinIdByTrackId()

template <typename _TrackT>
inline bool TracksTemplate<_TrackT>::hasLabel(LabelT const label) const {
    if (!_updated)
        _updateCache();
    return _trackIdLinIds.find(label) != _trackIdLinIds.end();
    //return _tracks.end() != std::find_if(this->_tracks.cbegin(),this->_tracks.cend(), [&label](const _TrackT& track) { return label == track.getLabel(); });
}

template <typename _TrackT>
inline typename TracksTemplate<_TrackT>::Base::const_iterator TracksTemplate<_TrackT>::findLabel(LabelT const label) const {
    return std::find_if(this->_tracks.cbegin(),this->_tracks.cend(), [&label](const _TrackT& track) { return label == track.getLabel(); });
} //...findLabel()

template <typename _TrackT>
inline typename TracksTemplate<_TrackT>::Base::iterator TracksTemplate<_TrackT>::findLabel(LabelT const label) {
    return std::find_if(this->_tracks.begin(),this->_tracks.end(), [&label](_TrackT const& track) { return label == track.getLabel(); });
} //...findLabel()

/**
 * \param[in] frameIds      Start[, inbetween] and end frame ids of sequence. Both start and end are inclusive!
 * \param[in] minOverlap    Minimum number of frames needed to be inside "frameIds" for a track to be selected.
 */
template <typename _TrackT>
void TracksTemplate<_TrackT>::selectByFrame( const FrameIdsT& frameIds, TracksTemplate<_TrackT> &outTracks, const SELECT_CONDITION condition, const FrameId minOverlap ) const
{
    const FrameId startFrameId = frameIds.at( 0 );
    const FrameId endFrameId   = frameIds.at( frameIds.size() - 1 );
    int trackId = 0;
    outTracks.clear();
    for ( auto const &track : this->_tracks )
    {
        FrameId overlap(0);

        bool add( (condition == ALL) ? true : false );
        //for ( auto const& frameId : frameIds )
        for ( FrameId frameId = startFrameId; frameId <= endFrameId; ++frameId )
        {
            const bool hasPoint = track.hasPoint(frameId);

            if ( hasPoint && (frameId >= startFrameId) && (frameId <= endFrameId) )
                ++overlap;

            if ( condition == ALL )
            {
                add &= hasPoint;
            }
            else if ( (condition == ANY) || (condition == MIN_OVERLAP) )
            {
                if ( hasPoint )
                {
                    add |= true;
                    // if seen enough evidence
                    if ( overlap >= minOverlap )
                        break;
                }
            }
            else
            {
                throw new Tracks_UntreatedSelectConditionException("did not prepare for this");
            }
        } //...for all frameIds

        if ( add && ((condition != MIN_OVERLAP) || (overlap >= minOverlap)) )
        {
            outTracks.addTrack( track );
        }

        ++trackId;
    } //...for all tracks
} //...selectByFrame()

template <typename _TrackT>
template <typename _InnerPointT>
bool TracksTemplate<_TrackT>::findClosestTrack( _TrackT& outTrack, const FrameId frameId, const _InnerPointT& targetPoint, Scalar distLimit ) const {
    const Track2D* tmp;

    Scalar minDist( std::numeric_limits<Scalar>::max() );
    for ( auto const& track : this->_tracks )
    {
        if ( track.hasPoint(frameId) )
        {
            const TrackPoint2D::Point2T& point = track.getPoint(frameId).getPoint();
            Scalar dist( distance(point,targetPoint) );
            if ( dist < minDist )
            {
                minDist = dist;
                tmp     = &track;
            }
        }
    }
    if ( minDist < distLimit )
    {
        outTrack = *tmp;
        return true;
    }
    else
        return false;

} //...findClosestTrack()

template <typename _TrackT>
void TracksTemplate<_TrackT>::getLongestTracks( const TracksTemplate<_TrackT> &tracks, int K, TracksTemplate<_TrackT>& out, const float trackDistThresh )
{
    TracksByDisplacement<_TrackT> sorted;
    getLongestTracks( tracks, K, sorted, trackDistThresh );
    out.clear();
    //for ( auto const& displacementAndTrack : sorted )
    for ( auto it = sorted.rbegin(); it != sorted.rend(); ++it )
    {
        out.addTrack( it->second );
    }
} //...getLongestTracks()

template <typename _TrackT>
void TracksTemplate<_TrackT>::getTracksByLabel( const TracksTemplate<_TrackT> &tracks, const std::vector<LabelT>& labels, TracksTemplate<_TrackT>& out ) {
    for (auto const& track : tracks) {
        if ( std::find( labels.begin(), labels.end(), track.getLabel() ) != labels.end() )
            out.addTrack( track );
    }
} //...getTracksByLabel()

template <typename _TrackT>
void TracksTemplate<_TrackT>::getLongestTracks(TracksTemplate<_TrackT> const& tracks, const int K,
                                               TracksByDisplacement<_TrackT>& out, const float trackDistThresh ) {
    for (size_t i = 0; i != tracks.getTrackCount(); ++i) {
        const _TrackT& track = tracks.getTrack(i);
        float displacement = 0.f;

        //for ( size_t j = 1; j < tracks[i].getPoints().size(); ++j )
        const typename _TrackT::PointT* prev = NULL;
        //FrameId prevFrame {static_cast<FrameId>(-1)};
        for (auto it = track.getPoints().begin(); it != track.getPoints().end(); ++it) {
            if (prev) {
                displacement += (it->second.getPoint() - prev->getPoint()).norm();
            }
            prev      = &it->second;
            //prevFrame = it->first;
        }

        if ((displacement > out.begin()->first)) {
            const auto newFrameIdAndTrackPoint = *tracks.getTrack(i).getPoints().begin();
            enum { ADD, SKIP, SWAP } add = ADD;

            typename TracksByDisplacement<_TrackT>::iterator it = out.begin(), closest = out.end();
            // if proximity based filtering needed
            if ( trackDistThresh > 0. ) // 23478, 23532
            {
                Scalar closestDist = std::numeric_limits<Scalar>::max();
                // check already selected tracks for any close-by starting points
                for ( ; (it != out.end()) && (add != SKIP); ++it )
                {
                    // get already chosen starting point
                    if ( !it->second.hasPoint(newFrameIdAndTrackPoint.first) )
                        break;

                    const auto  oldFrameIdAndTrackPoint = *it->second.getPoints().find( newFrameIdAndTrackPoint.first );
                    // get distance from that point
                    float dist = (newFrameIdAndTrackPoint.second.getPoint() - oldFrameIdAndTrackPoint.second.getPoint()).norm();

                    const bool close  = dist < trackDistThresh;
                    const bool longer = displacement > it->first;
                    if ( longer)
                    {
                        if ( close )
                        {
                            if ( dist < closestDist )
                            {
                                dist = closestDist;
                                closest = it;
                            }
                            add = SWAP;
                        }
                        else
                        {
                            add = ADD;
                        }
                    }
                    else if ( close )
                        add = SKIP;
                } //..for all already selected tracks
            } //...if there is thresholding

            // add, if still a candidate
            if ( add == SWAP )
            {
//                    if ( closest != out.end() )
//                        std::cout << "swapping " << tracks.getTrack(i).getLabel() << " for " << closest->second.getLabel() << std::endl;
//                    else
//                        throw new Tracks_GetLongest_KConditionNotMetException("bammmm");
                out.erase(closest);
            }
            if ( add == ADD || add == SWAP )
            {
//                    if ( add != SWAP )
//                        std::cout << "adding " << tracks.getTrack(i).getLabel() << std::endl;
                out.insert( std::make_pair(displacement,tracks.getTrack(i)) );
            }
        } //...if needed or long enough

        // clean up
        while ( out.size() > static_cast<size_t>(K) )
            out.erase( out.begin() );
    } //...for candiate tracks
} //...getLongestTracks()

template <typename _TrackT>
TrackIds2LinIds TracksTemplate<_TrackT>::getLinIds() const
{
    TrackIds2LinIds out;
    for ( LinId linId = 0; linId != this->size(); ++linId )
    {
        out.insert({this->getTrack(linId).getTrackId(),linId});
    }
    return out;
} //...TracksTemplate::getLinIds()

template <typename _TrackT>
inline TrackId TracksTemplate<_TrackT>::getMaxTrackId() const {
    return std::max_element(std::begin(_tracks),std::end(_tracks),[](_TrackT const& trackA,_TrackT const& trackB){
        return trackA.getTrackId() < trackB.getTrackId();
    })->getTrackId();
}

template <typename _TrackT>
inline void TracksTemplate<_TrackT>::_setDirty() const {
    _trackIdLinIds.clear();
    _updated = false;
}

/** \warning *NOT* called from GroupedTracks::_updateCache() to do it in one loop. */
template <typename _TrackT>
inline void TracksTemplate<_TrackT>::_updateCache() const {
    std::cout << "[TracksTemplate][" << __func__ << "] " << "starting..." << std::endl;
    for (LinId i = 0; i != static_cast<LinId>(_tracks.size()); ++i)
        _trackIdLinIds[_tracks[i].getTrackId()] = i;
    _updated = true;
    std::cout << "[TracksTemplate][" << __func__ << "] " << "finished..." << std::endl;
} //...TracksTemplate::getMaxTrackId()

template <typename _TrackT>
inline _TrackT& TracksTemplate<_TrackT>::getTrack(LinId const linTrackId)
{ return this->_tracks.at(linTrackId); }

template <typename _TrackT>
TracksTemplate<_TrackT>::TracksTemplate()
    : _sequenceLength(-1), _updated(true) {}


#if 0
template <typename _TrackT>
void TracksTemplate<_TrackT>::emplace_back(typename Base::value_type&& v) {
    addTrack(v);
}
#endif

template <typename _TrackT>
inline auto TracksTemplate<_TrackT>::erase(const_iterator elem) -> iterator {
    if (_updated) {
        auto iter = _trackIdLinIds.find(elem->getTrackId());
        if (iter == _trackIdLinIds.end()) {
            std::cerr << "[" << __func__ << "] " << "super bad...cache out of sync..." << std::endl;
        } else
            _trackIdLinIds.erase(iter);
    }
    return _tracks.erase(elem);
}

template <typename _TrackT>
bool TracksTemplate<_TrackT>::isUpdated() const
{ return _updated; }

#if 0
template <typename _TrackT>
    void TracksTemplate<_TrackT>::getShortestTracks( const TracksTemplate<_TrackT> &tracks, const int K, TracksByDisplacement<_TrackT>& out, const float trackDistThresh )
    {
        for ( size_t i = 0; i != tracks.getTrackCount(); ++i )
        {
            const _TrackT& track = tracks.getTrack(i);
            float displacement = 0.f;

            //for ( size_t j = 1; j < tracks[i].getPoints().size(); ++j )
            const typename _TrackT::PointT* prev = NULL;
            FrameId prevFrame(-1);
            for ( auto it = track.getPoints().begin(); it != track.getPoints().end(); ++it )
            {
                if ( prev )
                {
//                    if ( prevFrame + 1 != it->first )
//                        std::cerr << "[" << __func__ << "]: " << "skipping nonconsequtive " << prevFrame << "->" << it->first << std::endl;
//                    else
                    displacement += (it->second - *prev).norm();
                }
                prev      = &it->second;
                prevFrame = it->first;
            }

            if ( (out.size() < K) || (displacement < out.end()->first) )
            {
                const auto newFrameIdAndTrackPoint = *tracks.getTrack(i).getPoints().begin();
                bool       add                     = true;

                // if proximity based filtering needed
                if ( trackDistThresh > 0. )
                {
                    // check already selected tracks for any close-by starting points
                    for ( auto it = out.begin(); (it != out.end()) && add; ++it )
                    {
                        // get already chosen starting point
                        if ( !it->second.hasPoint(newFrameIdAndTrackPoint.first) )
                            break;

                        const auto  oldFrameIdAndTrackPoint = *it->second.getPoints().find( newFrameIdAndTrackPoint.first );
                        // get distance from that point
                        float       dist                    = (newFrameIdAndTrackPoint.second - oldFrameIdAndTrackPoint.second).norm();
                        // check, if distance small enough
                        if (    (newFrameIdAndTrackPoint.first == oldFrameIdAndTrackPoint.first)   // same frameID
                             && (dist                          <  trackDistThresh              )   // too close
                             && (displacement                  >  it->first                    ) )
                        {
                            std::cout << "[" << __func__ << "]: " << "skipping " << tracks.getTrack(i).getLabel() << " too close(" << dist << ")\n";
                            add = false;
                        } //...if needs skipping
                    } //..for all already selected tracks
                } //...if there is thresholding

                // add, if still a candidate
                if ( add )
                    out.insert( std::make_pair(displacement,tracks.getTrack(i)) );
            } //...if needed or long enough

            // clean up
            while ( out.size() > K )
                out.erase( out.begin() );
        } //...for candiate tracks
    } //...getLongestTracks()
#endif

// --------------------------------------------------------------- //
// -----------------------  FilterZFunctor ----------------------- //
// --------------------------------------------------------------- //

template <typename _Scalar,typename _Comparator>
bool FilterZFunctor<_Scalar,_Comparator>::operator() (Track3D const& track) const
{
    for ( auto const& frameIdAndPoint : track )
        //if ( frameIdAndPoint.second(2) > _truncDepth )
        if ( _comparator(frameIdAndPoint.second(2),_truncDepth) )
        {
            std::cout << "[" << __func__ << "]: " << "throwing away " << frameIdAndPoint.second.getPoint().transpose() << std::endl;
            return false;
        }
    return true;
}
} //...ns tracking

#endif // TRACKVIDEO_COMMON_TRACKS_HPP
