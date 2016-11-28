//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_COMMON_GROUPEDTRACKS_HPP
#define TRACKVIDEO_COMMON_GROUPEDTRACKS_HPP

#include "tracking/common/groupedTracks.h"

namespace tracking {

template <typename _TrackT>
inline typename GroupedTracks<_TrackT>::TrackT& GroupedTracks<_TrackT>::addTrack(TrackT const& track, GroupId const groupId) {
    Base::addTrack(track);
    _indices [groupId].insert(size()-1);
    if (_updated) {
        _trackIdGroupIds[track.getTrackId()] = groupId;
        //_trackIdLinIds[track.getTrackId()]   = size() - 1;
    }
    return _tracks.back();
} //...addTrack()

template <typename _TrackT>
inline typename GroupedTracks<_TrackT>::TrackT& GroupedTracks<_TrackT>::addTrack(TrackT &&track, GroupId const groupId) {
    Base::addTrack(std::move(track));
    _indices[groupId].insert(size()-1);
    if (_updated) {
        _trackIdGroupIds[track.getTrackId()] = groupId;
        //_trackIdLinIds[track.getTrackId()]   = size() - 1;
    }
    return _tracks.back();
} //...addTrack()

template <typename _TrackT>
inline GroupId GroupedTracks<_TrackT>::getGroupId(TrackId const trackId) const {
    if (!_updated)
        _updateCache();

    auto iter = _trackIdGroupIds.find(trackId);
    if (iter == _trackIdGroupIds.end())
        throw new GroupedTracks_NoGroupIdForTrackIdException(
            "Could not find groupId for track Id" + std::to_string(trackId));
    else
        return iter->second;

    #if 0
    else {
        for (GroupedTrackIds::value_type const& pair : _indices) {
            GroupId const groupId = pair.first;
            GroupT const      & group = pair.second;
            for (TrackId const& index : group)
                if (_tracks.at(index).template getLabel() == static_cast<typename Base::LabelT>(trackId))
                    return groupId;
        }
    }
    #endif
    throw new GroupedTracks_NoGroupIdForTrackIdException(
        "Could not find groupId for track Id" + std::to_string(trackId));
} //...getGroupId()

template <typename _TrackT>
inline GroupId GroupedTracks<_TrackT>::getGroupIdForLinId(LinId const linId) const {
    if (!_updated)
        _updateCache();
    TrackId const trackId = getTrack(linId).getTrackId();
    return getGroupId(trackId);
//    } else {
//        for (GroupedTrackIds::value_type const& pair : _indices) {
//            GroupId const groupId = pair.first;
//            GroupT const& group = pair.second;
//            if (group.find(linId) != group.end())
//                return groupId;
//        }
//    }
    throw new GroupedTracks_NoGroupIdForTrackIdException( "Could not find groupId for track Id" + std::to_string(linId) );
} //...getGroupId()

template <typename _TrackT>
inline void GroupedTracks<_TrackT>::getGroupAsSubset(GroupedTracks<_TrackT>& group, GroupId const groupId) const {
    group.clear();
    for (const TrackId &linId : _indices.at(groupId))
        group.addTrack(getTrack(linId), groupId);
} //...getGroupAsSubset()

template <typename _TrackT>
inline std::vector<_TrackT const*> GroupedTracks<_TrackT>::getTracksInGroup(GroupId const groupId) const {
    std::vector<_TrackT const*> pTracks;
    for (TrackId const linTrackId : _indices.at(groupId))
        pTracks.emplace_back( &getTrack(linTrackId) );

    return pTracks;
} //...getTracksInGroup()

template <typename _TrackT>
inline bool GroupedTracks<_TrackT>::isGrouped() const
{ return _indices.size(); }

template <typename _TrackT>
inline auto GroupedTracks<_TrackT>::getIndices() -> GroupedTrackIds const&
{ return _indices; }

template <typename _TrackT>
inline TrackId GroupedTracks<_TrackT>::getGroupCount() const
{ return _indices.size(); }

template <typename _TrackT>
inline TrackId GroupedTracks<_TrackT>::getGroupSize(TrackId const groupId) const
{ return _indices.at(groupId).size(); }

template <typename _TrackT>
inline _TrackT const& GroupedTracks<_TrackT>::getTrackFromGroup(GroupedTracks<_TrackT>::GroupConstIterator groupIt) const
{ return this->getTrack(*groupIt); }

template <typename _TrackT>
inline auto GroupedTracks<_TrackT>::getGroups() const -> GroupedTrackIds const&
{ return _indices; }

template <typename _TrackT>
inline bool GroupedTracks<_TrackT>::hasGroupId(GroupId const groupId) const
{ return _indices.find(groupId) != _indices.end(); }

template <typename _TrackT>
inline void GroupedTracks<_TrackT>::clear() {
    Base::clear();
    _indices.clear();
    _setDirty();
}

template <typename _TrackT>
inline void GroupedTracks<_TrackT>::_setDirty() const {
    Base::_setDirty();
    _trackIdGroupIds.clear();
    _updated = false;
}

template <typename _TrackT>
inline void GroupedTracks<_TrackT>::_updateCache() const {
    std::cout << "[GroupedTracks][" << __func__ << "] " << "starting..." << std::endl;
    // not calling Base::_updateCache to save time
    for (auto const& groupIdLinIds : _indices) {
        for (LinId const& linId : groupIdLinIds.second) {
            TrackId const trackId = getTrack(linId).getTrackId();
            _trackIdLinIds[trackId]   = linId;
            _trackIdGroupIds[trackId] = groupIdLinIds.first;
        }
    }
    _updated = true;
    std::cout << "[GroupedTracks][" << __func__ << "] " << "finished..." << std::endl;
}

template <typename _TrackT> typename GroupedTracks<_TrackT>::iterator
GroupedTracks<_TrackT>::erase(const_iterator elem) {
    if (elem == _tracks.end())
        _tracks.end();

    LinId const linId = getLinIdByTrackId(elem->getTrackId());
    if (_updated) {
        _trackIdGroupIds.erase(elem->getTrackId());
    }

    bool found = false;
    for (std::pair<const GroupId, GroupT> &group : _indices) {
        auto iter = std::find(group.second.begin(), group.second.end(), linId);
        if (iter != group.second.end()) {
            group.second.erase(iter);
            found = true;
            break; // assuming there's only one...
        }
    }

    if (!found)
        LogicException("Ungrouped track found...something is not right...", __FILE__, __LINE__);

    return TracksTemplate<_TrackT>::erase(elem);
} //...erase()

template <typename _TrackT> typename GroupedTracks<_TrackT>::iterator
GroupedTracks<_TrackT>::eraseByTrackId(TrackId const trackId)
{ return erase(findLabel(trackId)); }

//template <typename _TrackT>
//inline Tracks3D& GroupedTracks<_TrackT>::asTracks()
//{ return *reinterpret_cast<Tracks3D*>(this); }

} //...ns tracking

#endif //TRACKVIDEO_COMMON_GROUPEDTRACKS_HPP
