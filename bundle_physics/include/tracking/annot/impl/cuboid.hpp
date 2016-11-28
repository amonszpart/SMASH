#ifndef TV_CUBOID_HPP
#define TV_CUBOID_HPP

#include "tracking/annot/cuboid.h"
#include <string>
#include "tracking/vis/visualizer.h"
#include "tracking/vis/impl/visualizer.hpp"

namespace tracking {
  namespace bundle_physics {
    template<typename _VisT, typename _Vector3, typename _TransformationT>
    inline void drawCuboid(_VisT &vis, const _TransformationT &T, const _Vector3 &color, const std::string &nameArg, std::vector<_Vector3> *outCorners) {
        //std::vector<cv::Scalar> colors = { {225,165,6}, {98,179,51}, {140,149,148}, {247,45,163}, {248,59,53}, {18,202,254}, {101,85,207}, {101,85,207}, {1,163,165}, {1,163,165},  };
        static const std::vector<cv::Scalar> colors = {{240, 163, 255},
                                                       {0,   117, 220},
                                                       {153, 63,  0},
                                                       {76,  0,   92},
                                                       {25,  25,  25},
                                                       {0,   92,  49},
                                                       {43,  206, 72},
                                                       {255, 204, 153},
                                                       {100, 128, 128},
                                                       {148, 255, 181},
                                                       {143, 124, 0},
                                                       {157, 204, 0},
                                                       {194, 0,   136},
                                                       {0,   51,  128},
                                                       {255, 164, 5},
                                                       {255, 168, 187},
                                                       {66,  102, 0},
                                                       {255, 0,   16},
                                                       {94,  241, 242},
                                                       {0,   153, 143},
                                                       {224, 255, 102},
                                                       {116, 10,  255},
                                                       {153, 0,   0},
                                                       {255, 255, 128},
                                                       {255, 255, 0},
                                                       {255, 80,  5}};

        char name[512];
        Cuboid::CornersT corners  = *Cuboid::getCorners();
        for (int col = 0; col != Cuboid::getCorners()->cols(); ++col) {
            corners.col(col) = T * Cuboid::getCorners()->col(col);
            if (outCorners) {
                outCorners->push_back(corners.col(col).head<3>());
            }
        }

        for (int edgeId = 0; edgeId != int(Cuboid::getEdges().size()); edgeId += 2) {
            sprintf(name, "%s_edge_%d", nameArg.c_str(), edgeId);
            const auto &curr = colors.at(edgeId / 2 % colors.size());

            vis.addLine(corners.col(Cuboid::getEdges().at(edgeId)).template head<3>(), corners.col(Cuboid::getEdges().at(edgeId + 1)).template head<3>(), _Vector3(color(0) * .4 + curr[0] / 255. * .6,
                color(1) * .4 + curr[1] / 255. * .6, color(2) * .4 + curr[2] / 255. * .6), name, 3.f, 0.8f);
        }
        //auto center = Cuboid::TransformationT(T).translation();
        //vis.addLine( center, center + Cuboid::TransformationT(T).rotation() * Vector3::UnitX(), Vector3(1.,0.,0.), "cubX", 1., 1. );
    } //...drawCuboid
  } //...ns bundle_physics
} //...ns tracking

#endif // TV_CUBOID_HPP

