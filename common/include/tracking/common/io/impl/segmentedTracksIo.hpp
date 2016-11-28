//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_SEGMENTED_HPP
#define TRACKVIDEO_SEGMENTED_HPP

#include "tracking/common/io/segmentedTracksIo.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include <fstream>

#define READ_LINE(ifstream, line) \
        {\
            if (!std::getline( ifstream, line ))\
            {\
                std::cerr << "could not read line after " << line << std::endl;\
                return EXIT_FAILURE;\
            }\
        }

namespace tracking {
  namespace io {
    namespace internal {
      template <typename _TrackT>
      void parseLine(std::string const& line, _TrackT& point, FrameId& frameId);
      template <> void parseLine(const std::string& line, TrackPoint2D& point, FrameId& frameId) {
          std::istringstream(line) >> point(0) >> point(1) >> frameId;
      }
      template <> void parseLine(std::string const& line, TrackPoint3D& point, FrameId& frameId) {
          std::istringstream(line) >> point.getPoint()(0) >> point.getPoint()(1) >> frameId >> point.getPoint()(2);
      }
    }

    template <typename _TrackT>
    int readSegmentedTracks(std::string const &path, GroupedTracks<_TrackT> &tracks) {
        std::ifstream ifstream(path);
        if (!ifstream.is_open()) {
            std::cerr << __func__ << " Could not open " << path << std::endl;
            return EXIT_FAILURE;
        }

        std::string line;

        // nFrames
        GroupId nFrames(0u);
        {
            READ_LINE(ifstream, line);
            std::istringstream(line) >> nFrames;
        }

        // nTracks
        TrackId nTracks(0u);
        {
            READ_LINE(ifstream, line);
            std::istringstream(line) >> nTracks;
            std::cout << "[" << __func__ << "] " << "reading " << nTracks << " tracks" << std::endl;
        }

        TrackId trackId(0u);
        while (std::getline(ifstream, line)) {
            std::istringstream iss(line);

            // GroupId
            GroupId groupId(-1u);
            iss >> groupId;

            // trackLength
            TrackId trackLength(0);
            iss >> trackLength;

            TrackId pId;
            _TrackT track {trackId};
            for (pId = 0; pId != trackLength; ++pId) {
                typename _TrackT::PointT point;
                FrameId      frameId;
                READ_LINE(ifstream, line);
                internal::parseLine( line, point, frameId );
                track.addPoint(frameId, point);
            }

            tracks.addTrack(track, groupId);
            //std::cout << "added " << tracks.back() << std::endl;

            ++trackId;
        }

        return EXIT_SUCCESS;
    } //...readSegmentedTracks()

  } //...ns io
} //..ns tracking

#undef READ_LINE

#endif //TRACKVIDEO_SEGMENTED_HPP
