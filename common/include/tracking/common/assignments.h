//
// Created by bontius on 07/03/16.
//

#ifndef TRACKVIDEO_COMMON_ASSIGNMENTS_H
#define TRACKVIDEO_COMMON_ASSIGNMENTS_H

#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/util/exception.h"
#include "tracking/common/typedefs.h"
#include <map>

namespace tracking {

DEFINE_EXCEPTION(TracksToJoints_PartNotFound)
/** \brief Stores assignments of points to joints (and rigid parts). */
template <typename _TargetId, size_t _InvalidId>
class TracksToTarget {
public:
    typedef _TargetId TargetId;
    enum {InvalidId = _InvalidId};
    typedef std::map     <TrackId,_TargetId> BaseT;
    typedef std::multimap<_TargetId,TrackId> InverseT;

    explicit TracksToTarget() = default;
    virtual ~TracksToTarget() = default;

    /** \brief Appends groupToJoint assignment to assignments. */
    static void assignGroupToTarget(TracksToTarget &assignments, GroupId const groupId, TargetId const jointId, GroupedTracks2d const &tracks2d);
    /** \brief Creates and inverse object with tracks grouped by their assigned joint. */
    static InverseT invert(TracksToTarget const& assignments);
    /** \copydoc invert */
    inline InverseT invert() const { return invert(*this); }

    inline typename BaseT::mapped_type      & operator[](typename BaseT::key_type const&  key)
    { return _base[key];      }
    inline typename BaseT::mapped_type      & operator[](typename BaseT::key_type      && key)
    { return _base[key];      }
    inline typename BaseT::mapped_type const& at        (typename BaseT::key_type const&  key) const
    { return _base.at(key);   }
    inline typename BaseT::mapped_type      & at        (typename BaseT::key_type const&  key)
    { return _base.at(key);   }
    inline typename BaseT::iterator           begin     ()
    { return _base.begin();   }
    inline typename BaseT::iterator           end       ()
    { return _base.end();     }
    inline typename BaseT::const_iterator     begin     () const
    { return _base.begin();   }
    inline typename BaseT::const_iterator     end       () const
    { return _base.end();     }
    inline typename BaseT::const_iterator     cbegin    () const
    { return _base.cbegin();  }
    inline typename BaseT::const_iterator     cend      () const
    { return _base.cend();    }
    inline typename BaseT::iterator           find      (typename BaseT::key_type const&  key )
    { return _base.find(key); }
    inline typename BaseT::const_iterator     find      (typename BaseT::key_type const&  key ) const
    { return _base.find(key); }
    inline typename BaseT::size_type          size      () const
    { return _base.size(); }

    inline std::pair<typename BaseT::iterator, bool> insert(typename BaseT::value_type const&  pair)
    { return _base.insert(pair); }
    inline std::pair<typename BaseT::iterator, bool> insert(typename BaseT::value_type      && pair)
    { return _base.insert(pair); }
    inline void                                      insert(typename BaseT::const_iterator begin, typename BaseT::const_iterator end)
    { return _base.insert(begin,end); }

    inline typename BaseT::iterator           erase(typename BaseT::const_iterator iter)
    { return _base.erase(iter); }

protected:
    BaseT _base;
};
} //...ns tracking

#endif //TRACKVIDEO_COMMON_ASSIGNMENTS_H
