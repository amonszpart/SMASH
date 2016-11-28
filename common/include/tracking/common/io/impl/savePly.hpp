//
// Created by bontius on 30/05/16.
//

#ifndef TRACKVIDEO_COMMON_SAVEPLY_HPP
#define TRACKVIDEO_COMMON_SAVEPLY_HPP

#include "tracking/common/io/savePly.h"
#include "tracking/common/util/colors.h"
#include <iostream>
#include <fstream>

namespace tracking {
  namespace io {
    template<typename _Tracks3dT>
    int savePly(std::string const& path, _Tracks3dT const& tracks3d, std::map<TrackId, LinId>* const linIdsArg,
                std::map<TrackId, Eigen::Vector3f> const* const trackColors) {
        std::map<TrackId, LinId> linIds;

        auto colors = colors::paletteMediumColoursCv(tracks3d.getGroupCount(), true);

        std::ofstream file(path.c_str());
        if (!file.is_open()) {
            std::cerr << "[" << __func__ << "]: " << "could not open " << path << std::endl;
            return EXIT_FAILURE;
        }

        file << "ply\n"
             << "format ascii 1.0\n"
             << "comment Aron generated\n"
             << "element vertex " << tracks3d.size() << "\n"
             << "property float x\n"
             << "property float y\n"
             << "property float z\n"
             << "property uchar red\n"
             << "property uchar green\n"
             << "property uchar blue\n"
             << "end_header\n";

        for (auto const& pair : tracks3d.getGroups()) {
            for (TrackId const linId : pair.second) {
                auto const& track3D = tracks3d.getTrack(linId);
                auto const& p       = track3D.getFirstPoint();
                if (trackColors) {
                    Eigen::Vector3f rgb(Eigen::Vector3f::Zero());
                    auto it = trackColors->find(track3D.getTrackId());
                    if (it != trackColors->end()) {
                        rgb = it->second;
                    }
                    file << p.getPoint()(0) << " " << p.getPoint()(1) << " " << p.getPoint()(2) << " "
                         << static_cast<int>(round(rgb(0))) << " " << static_cast<int>(round(rgb(1))) << " "
                         << static_cast<int>(round(rgb(2))) << std::endl;
                } else {
                    auto const& color = colors.at(pair.first % colors.size());
                    file << p.getPoint()(0) << " " << p.getPoint()(1) << " " << p.getPoint()(2) << " " << color(0)
                         << " " << color(1) << " " << color(2) << std::endl;
                }
                if (linIdsArg) {
                    linIds.insert({linId, track3D.getTrackId()});
                }
            }
        }

        file.close();

        if (linIdsArg) {
            *linIdsArg = linIds;
        }
        return EXIT_SUCCESS;
    } //...savePly()
  } //...ns io
} //..ns tracking

#endif //TRACKVIDEO_COMMON_SAVEPLY_HPP
