//
// Created by bontius on 28/01/16.
//

#include "tracking/common/trackPoint2d.h"
#include "tracking/common/trackPoint3d.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/io/impl/segmentedTracksIo.hpp"

#include <fstream>

namespace tracking {
  namespace io {

    int writeSegmentedTracks(GroupedTracks2d const &tracks, std::string const &path) {
        std::ofstream f(path);
        if (!f.is_open()) {
            std::cerr << "could not open " << path << " for writing" << std::endl;
            return EXIT_FAILURE;
        }

        //f << tracks.getGroupCount() << std::endl;
        f << '0' << std::endl;
        f << tracks.size() << std::endl;

        for (auto const &group : tracks.getGroups()) {
            GroupId const groupId = group.first;
            for (auto const &linTrackId : group.second) {
                auto const &track = tracks.getTrack(linTrackId);

                // groupId nPoints
                f << groupId << " " << track.size() << std::endl;
                for (auto const &pair : track.getPoints()) {
                    // x y frameId
                    f << pair.second(0) << " " << pair.second(1) << " " << pair.first << std::endl;
                }
            }
        }

        f.close();
        std::cout << "wrote to " << path << std::endl;
        return EXIT_SUCCESS;
    } //...writeSegmentedTracks()

    template int readSegmentedTracks( const std::string&, GroupedTracks3d &);
    template int readSegmentedTracks( const std::string&, GroupedTracks2d&);

  } //...ns io
} //...ns tracking