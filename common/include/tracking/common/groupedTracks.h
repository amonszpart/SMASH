//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_COMMON_GROUPEDTRACKS_H
#define TRACKVIDEO_COMMON_GROUPEDTRACKS_H

#include "tracking/common/tracks.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/util/exception.h"
#include "tracking/common/typedefs.h"
#include <set>

namespace tracking {

DEFINE_EXCEPTION(GroupedTracks_NoGroupIdForTrackId)
DEFINE_EXCEPTION(GroupedTracks_UpdateCacheUnimplemented)

template <typename _TrackT>
class GroupedTracks : public TracksTemplate<_TrackT> {
protected:
    enum { NeedsToAlign = (sizeof(_TrackT)%16)==0 };
public:
    typedef _TrackT                                                 TrackT;
    typedef TracksTemplate<_TrackT>                                 Base;

    typedef std::set<LinId>                                         GroupT;
    typedef std::map<GroupId, GroupT>                               GroupedTrackIds;
    typedef typename GroupedTrackIds::mapped_type::const_iterator   GroupConstIterator;
    using typename Base::iterator;
    using typename Base::const_iterator;

    static constexpr GroupId INVALID_GROUP_ID = GroupId {static_cast<GroupId>(-1)};

    bool                        isGrouped         ()                                            const;
    TrackId                     getGroupCount     ()                                            const;
    TrackId                     getGroupSize      (TrackId const groupId)                       const;
    TrackT          const&      getTrackFromGroup (GroupConstIterator groupIt)                  const;
    GroupedTrackIds const&      getGroups         ()                                            const;
    GroupId                     getGroupId        (TrackId const  trackId)                      const;
    bool                        hasGroupId        (GroupId const groupId)                       const;
    GroupId                     getGroupIdForLinId(LinId   const  linId  )                      const;
    void                        getGroupAsSubset  (GroupedTracks &group, GroupId const groupId) const;
    std::vector<_TrackT const*> getTracksInGroup  (const GroupId groupId)                       const;

    void                        clear             ();
    TrackT&                     addTrack          (TrackT  const& track, GroupId const groupId);
    TrackT&                     addTrack          (TrackT      && track, GroupId const groupId);
    GroupedTrackIds const&      getIndices        ();
    //Tracks3D             &      asTracks          ();

    virtual iterator erase(const_iterator elem) override;
    iterator eraseByTrackId(TrackId const trackId);

    using                 Base::getTrack;
    using                 Base::size;
    using                 Base::findLabel;
    using                 Base::getLinIdByTrackId;

protected:
    GroupedTrackIds                    _indices;
    using                        Base::_tracks;

    // speed optimization:
    mutable std::map<TrackId, GroupId> _trackIdGroupIds;
    using                        Base::_trackIdLinIds;
    using                        Base::_updated;

    virtual void _setDirty() const override final;
    virtual void _updateCache() const override final;

    _TrackT& addTrack(_TrackT const& track) = delete;
    _TrackT& addTrack(_TrackT     && track) = delete;

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
}; //...class GroupedTracks

} //...ns tracking

#endif //TRACKVIDEO_COMMON_GROUPEDTRACKS_H
