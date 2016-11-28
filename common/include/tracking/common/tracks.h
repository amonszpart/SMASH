#ifndef TRACKVIDEO_COMMON_TRACKS_H
#define TRACKVIDEO_COMMON_TRACKS_H

#include "tracking/common/tracksFwDecl.h"
#include "tracking/common/track.h"
#include "tracking/common/util/exception.h"
#include "tracking/common/typedefs.h"
#include <vector>
#include <map>

/** \ingroup common */

namespace tracking {

template <typename _TrackT>
struct TracksByDisplacement : public std::multimap<float,_TrackT> {
    typedef std::multimap<float,_TrackT> Base;
    using Base::Base;
};
typedef TracksByDisplacement<Track2D> Tracks2DByDisplacement;

DEFINE_EXCEPTION(Tracks_UntreatedSelectCondition)
DEFINE_EXCEPTION(Tracks_GetTrackByLabel_NoSuchLabel)
DEFINE_EXCEPTION(Tracks_GetTrackByLabel_CacheMismatch)
DEFINE_EXCEPTION(Tracks_GetTrackByTrackId_CacheMismatch)
DEFINE_EXCEPTION(Tracks_GetLongest_KConditionNotMet)
DEFINE_EXCEPTION(Tracks_AddTrack_CacheMismatch)


/** \brief A list of tracks (2D or 3D).
 *  \tparam _TrackT Type of track stored. Concept: \ref Track2D or \ref Track3D.
 */
template <typename _TrackT>
class TracksTemplate {
protected:
    enum { NeedsToAlign = (sizeof(_TrackT)%16)==0 };
    typedef std::vector<_TrackT>          Base;
    typedef typename Base::iterator       iterator;
    typedef typename Base::const_iterator const_iterator;

public:
    typedef          _TrackT            TrackT;
    typedef typename _TrackT::LabelT    LabelT;
    typedef typename _TrackT::Scalar    Scalar;
    typedef size_t                      CountT;
    enum SELECT_CONDITION { ANY, ALL, MIN_OVERLAP };

    TracksTemplate();
    TracksTemplate(TracksTemplate const&) = default;
    TracksTemplate(TracksTemplate &&) = default;
    TracksTemplate& operator=(TracksTemplate const&) = default;
    TracksTemplate& operator=(TracksTemplate &&) = default;
    virtual ~TracksTemplate() = default;

    inline          void                         setSequenceLength(size_t const seqLen)              { _sequenceLength = seqLen; }
    inline          CountT                       getSequenceLength()                           const { return this->_sequenceLength;   }
    inline          CountT                       getTrackCount    ()                           const { return this->_tracks.size();      }
    inline          CountT                       size             ()                           const { return this->_tracks.size(); }
    inline          _TrackT const&               getTrack         (LinId   const linTrackId)   const { return this->_tracks.at(linTrackId); }
    _TrackT&                     getTrack         (LinId   const linTrackId);
    /*           */ _TrackT const&               getTrackByLabel  (LabelT  const label)        const;
    /*           */ _TrackT&                     getTrackByLabel  (LabelT  const label);
    /*           */ LinId                        getLinIdByLabel  (LabelT  const label)        const;
    /*           */ LinId                        getLinIdByTrackId(TrackId const trackId)      const;
    /*           */ bool                         hasLabel         (LabelT  const label)        const;
    /*           */ _TrackT&                     addTrack         (_TrackT const& track);
    /*           */ _TrackT&                     addTrack         (_TrackT     && track);
    inline typename Base::const_iterator         begin            ()                           const { return _tracks.begin(); }
    inline iterator begin            ()                                 { _setDirty(); return _tracks.begin(); }
    inline typename Base::const_reverse_iterator rbegin           ()                           const { return _tracks.rbegin(); }
    inline typename Base::reverse_iterator       rbegin           ()                                 { _setDirty(); return _tracks.rbegin(); }
    inline typename Base::const_iterator         end              ()                           const { return _tracks.end(); }
    inline iterator end              ()                                 { _setDirty(); return _tracks.end(); }
    inline typename Base::const_reverse_iterator rend             ()                           const { return _tracks.rend(); }
    inline typename Base::reverse_iterator       rend             ()                                 { _setDirty(); return _tracks.rend(); }
    inline typename Base::const_reference        at               (typename Base::size_type i) const { return _tracks.at(i); }
    inline typename Base::reference              at               (typename Base::size_type i)       { _setDirty(); return _tracks.at(i); }
    inline          void                         clear            ()                                 { _setDirty(); _tracks.clear(); _sequenceLength = CountT(-1); }
    inline typename Base::const_reference        back             ()                           const { return _tracks.back(); }
    inline typename Base::reference              back             ()                                 { _setDirty(); return _tracks.back(); }

//    virtual void                                 emplace_back     (typename Base::value_type&& v);

    virtual iterator erase            (const_iterator elem);


    void selectByFrame(FrameIdsT const& frameIds, TracksTemplate& outTracks, const SELECT_CONDITION condition,
                       const FrameId minOverlap) const;

    template <typename _InnerPointT>
    bool
    findClosestTrack(_TrackT& outTrack, const FrameId frameId, const _InnerPointT& pnt, Scalar distLimit) const;

    /** \brief Tries to find track by trackId. */
    typename Base::const_iterator                 getTrackByTrackId(TrackId const trackId) const;

    typename Base::const_iterator                 findLabel       (LabelT const label) const;
    iterator findLabel       (LabelT const label);

    TrackIds2LinIds                               getLinIds       () const;
    TrackId                                       getMaxTrackId   () const;


    static          void                          getLongestTracks(TracksTemplate const& tracks, int K, TracksByDisplacement<_TrackT>& out, const float trackDistThresh = 0.f);
    static          void                          getLongestTracks(TracksTemplate const& tracks, int K, TracksTemplate& out, const float trackDistThresh = 0.f );
    static          void                          getTracksByLabel(TracksTemplate const& tracks, std::vector<LabelT> const& labels, TracksTemplate& out );
    template <typename _FunctorT>
    static          void                          filterTracks    (TracksTemplate      & tracks, _FunctorT const& functor );

    bool isUpdated() const;

protected:
    CountT  _sequenceLength;  //!< Max frameId in video.
    Base    _tracks;          //!< List of tracks (main functionality).

    // speed optimization:
    virtual void _setDirty() const;
    virtual void _updateCache() const;

    mutable bool                       _updated;
    mutable std::map<TrackId, LinId>   _trackIdLinIds;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
}; //...cls Tracks

template <typename _Scalar,typename _Comparator>
struct FilterZFunctor {
    FilterZFunctor( _Scalar truncDepth, _Comparator cmp = std::greater<_Scalar>()) : _truncDepth(truncDepth), _comparator(cmp) {}
    bool operator() ( const Track3D& track ) const;

    _Scalar            _truncDepth;
    _Comparator const& _comparator;
};
} //...ns tracking

#endif // TRACKVIDEO_COMMON_TRACKS_H
