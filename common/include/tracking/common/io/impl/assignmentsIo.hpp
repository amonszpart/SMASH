//
// Created by bontius on 25/05/16.
//

#ifndef TRACKVIDEO_COMMON_ASSIGNMENTSIO_HPP
#define TRACKVIDEO_COMMON_ASSIGNMENTSIO_HPP

#include "tracking/common/io/assignmentsIo.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "picojson/picojson.h"
#include <fstream>
#include <iostream>
#include <set>

namespace tracking{
  namespace io {
    DEFINE_EXCEPTION(ReadAssignments_NoJointIdField)
    DEFINE_EXCEPTION(ReadAssignments_NoIdsField)

    template <typename _TargetId, size_t _InvalidId>
    int readAssignments(TracksToTarget<_TargetId, _InvalidId>& assignments, std::string const& assignmentsPath,
                        GroupedTracks2d const& tracks2d, std::string const& targetTag)
    {
        // open input text file
        std::ifstream inFile;
        inFile.open(assignmentsPath.c_str());
        if (!inFile.is_open())
        {
            std::cerr << "[" << __func__ << "]: could not open " << assignmentsPath << std::endl;
            return EXIT_FAILURE;
        }

        picojson::value jsonData;
        std::string     err = picojson::parse(jsonData, inFile);
        if (!err.empty())
        {
            std::cerr << "[" << __func__ << "]: could not parse " << err << std::endl;
            return EXIT_FAILURE;
        }

        int entryIndex = 0;
        while (jsonData.contains(entryIndex))
        {
            picojson::object object = jsonData.get(entryIndex).template get<picojson::object>();

            if (object.find(targetTag) == object.end())
                throw new ReadAssignments_NoJointIdFieldException("Need field \"" + targetTag + "\"");
            _TargetId targetId = std::atol(object.at(targetTag).to_str().c_str());

            if (object.find("ids") == object.end())
                throw new ReadAssignments_NoIdsFieldException("Need field \"ids\"");
            picojson::array   array = object.at("ids").get<picojson::array>();
            std::set<TrackId> ids;
            for (picojson::value const &v : array)
            {
                ids.insert(static_cast<TrackId>(std::atof(v.to_str().c_str())));
            }

            // group
            if ((object.find("type") != object.end()) && (object.at("type").to_str().compare("group") == 0))
            {
                for (TrackId const &groupId : ids)
                {
                    std::vector<Track2D const *> tracks = tracks2d.getTracksInGroup(groupId);
                    for (Track2D const *const &track : tracks)
                        assignments[track->getTrackId()] = targetId;
                }
            }
            else
                for (TrackId const &trackId : ids)
                    assignments[trackId] = targetId;

            ++entryIndex;
        } //...for each object

        return EXIT_SUCCESS;
    } //...readAssignments()

    /** \param[in ] saveLinear Save linear track id (\ref LinId) for each track instead of unique \ref TrackId. */
    template <typename _TargetId, size_t _InvalidId>
    int writeAssignments(TracksToTarget<_TargetId, _InvalidId> const& assignments, std::string const& assignmentsPath,
                         TrackIds2LinIds const* const trackIds2LinIds, std::string const& targetTag) {
        std::ofstream f(assignmentsPath);
        if (!f.is_open())
        {
            std::cerr << "[" << __func__ << "] could not open " << assignmentsPath << " for writing..." << std::endl;
            return EXIT_FAILURE;
        }

        picojson::array jAssignments;

        auto    tracksByJoint = TracksToTarget<_TargetId,_InvalidId>::invert(assignments);
        auto    store         = [&targetTag](picojson::object &jAss, picojson::array const &jIds, _TargetId const &targetId, picojson::array &jAssignments)
        {
            jAss[targetTag] = picojson::value(static_cast<int64_t>(targetId));
            jAss["type"]    = picojson::value("tracks");
            jAss["ids"]     = picojson::value(jIds);
            jAssignments.push_back(picojson::value(jAss));
        };
        _TargetId targetId    = _InvalidId;
        std::unique_ptr<picojson::object> jAss(new picojson::object());
        std::unique_ptr<picojson::array>  jIds(new picojson::array());
        for (auto const &pair : tracksByJoint)
        {
            TrackId const& trackId = pair.second;

            if (targetId == _InvalidId)
                targetId = pair.first;
            else if (targetId != pair.first)
            {
                store(*jAss, *jIds, targetId, jAssignments);
                jAss.reset(new picojson::object);
                jIds.reset(new picojson::array);
                targetId = pair.first;
            }

            if (trackIds2LinIds)
            {
                auto it = trackIds2LinIds->find(trackId);
                if ( it == trackIds2LinIds->end() )
                    std::cerr << "[" << __func__ << "] could not find linearid for trackId " << pair.second << std::endl;
                else
                    jIds->push_back(picojson::value(static_cast<int64_t>(it->second)));
            }
            else
                jIds->push_back(picojson::value(static_cast<int64_t>(trackId)));
        } //...for each assignment

        if (jIds->size())
        {
            store(*jAss, *jIds, targetId, jAssignments);
        }

        f << picojson::value(jAssignments).serialize();
        f.close();
        std::cout << "wrote to " << assignmentsPath << std::endl;

        return EXIT_SUCCESS;
    } //...writeAssignments()
  } //...ns io
} //...ns tracking


#endif //TRACKVIDEO_COMMON_ASSIGNMENTSIO_HPP
