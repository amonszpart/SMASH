//
// Created by bontius on 04/01/16.
//

#ifndef TRACKVIDEO_PARTUTIL_H
#define TRACKVIDEO_PARTUTIL_H

#include "tracking/phys/typedefs.h"
#include "tracking/common/util/exception.h"
#include "tracking/common/typedefs.h"

namespace tracking
{
  namespace bundle_physics
  {
    inline PartId getPartId(const FrameId frameId, const FrameIdsT &frameIds)
    {
        PartId partId(0);

        while ((partId + 2 < static_cast<PartId>(frameIds.size())) && (frameId > frameIds.at(partId + 1)))
        {
            //std::cout << "partId: " << partId << " +2 < " << frameIds.size() << " and " << frameId << "<" << frameIds.at(partId + 1) << std::endl;
            ++partId;
        }
        //std::cout << "partId " << partId << " for frameId " << frameId << std::endl;
        return partId;
    }

    DEFINE_EXCEPTION(BundleWithPhysics_GetPartStart_OutOfBounds)

    /** \brief Estimates first valid frameId of the parabola part.
     *  This is probably the one just after the collision or the start of the sequence.
     */
    inline FrameId getPartStart(const PartId partId, const FrameIdsT &frameIds)
    {
        if (partId >= static_cast<PartId>(frameIds.size()) || partId < PartId(0))
            throw new BundleWithPhysics_GetPartStart_OutOfBoundsException("");
        return frameIds.at(partId);
    }

    /** \brief Estimates last valid frameId of the parabola part.
     *  This is probably the one just before the collision or the end of the sequence.
     */
    inline FrameId getPartEnd(const PartId partId, const FrameIdsT &frameIds)
    {
        if (partId + 1 >= static_cast<PartId>(frameIds.size()) || partId < PartId(0))
            throw new BundleWithPhysics_GetPartStart_OutOfBoundsException("");
        return frameIds.at(partId + 1);
    }

    inline PartId getPartIdBefore(const CollId collId) { return collId; }
    inline PartId getPartIdAfter(const CollId collId) { return collId + 1; }

    DEFINE_EXCEPTION(BundleWithPhysics_GetOther_UnpreparedForMoreThan2Participants)
    template<typename _ParticipantsT>
    inline CuboidId getOther(const CuboidId &cuboidId, const _ParticipantsT &participants)
    {
        if (participants.size() > 2)
            throw new BundleWithPhysics_GetOther_UnpreparedForMoreThan2ParticipantsException("");
        if (cuboidId == participants.at(0))
            return participants.at(1);
        else
            return participants.at(0);
    } //...getOther()

    template<typename _ParticipantsT>
    inline CuboidId getParticipant(const int firstOrSecond, const CollId /*collId*/, const _ParticipantsT &participants) {
        if (participants.size() > 2 || firstOrSecond > 1)
            throw new BundleWithPhysics_GetOther_UnpreparedForMoreThan2ParticipantsException("");
        return participants.at(firstOrSecond);
    } //...getOther()

  } //...ns bundle_phsyics
} //...ns tracking


#endif //TRACKVIDEO_PARTUTIL_H
