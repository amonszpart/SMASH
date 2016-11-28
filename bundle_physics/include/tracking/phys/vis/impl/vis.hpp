//
// Created by bontius on 28/03/16.
//

#ifndef TRACKVIDEO_PHYS_VIS_HPP
#define TRACKVIDEO_PHYS_VIS_HPP

#include "tracking/annot/cuboid.h"

namespace tracking {
  namespace bundle_physics {

    template<typename _VisT>
    void drawCuboidStates2(const Cuboid &cuboid, _VisT &vis, const Scalar scale, const Vector3 &color, const int cuboidId, const FrameIdsT *const frameIds, bool const posOnly) {
        char name[512];
        for (auto const &frameAndState : cuboid.getStates()) {
            const FrameId &frameId = frameAndState.first;
//            const PoseLoc &state   = frameAndState.second;
            if (frameIds && (frameId < frameIds->front() || frameId > frameIds->back()))
                continue;
            sprintf(name, "cuboid%d_time%u", cuboidId, frameId);
            if (!cuboid.getState(frameId).hasPos())
                continue;
            if (!posOnly && cuboid.hasPose(frameId)) {
                auto transform = cuboid.getTransform4x4(frameId);
                transform.matrix().template block<3, 1>(0, 3) *= scale;

//                vis.addSphere(cuboid.getPosition(frameId), 0.025*scale, color, name);
                drawCuboid(vis, transform, color, name);
            } else if (cuboid.getState(frameId).hasPos()){
               vis.addSphere(cuboid.getPosition(frameId), 0.025*scale, color, name);
            }
        }
    } //...drawCuboidStates

    template<typename _VisT>
    void drawCuboidStates(const Cuboid &cuboid, _VisT &vis, const Scalar scale, const Vector3 &color, const int cuboidId, const FrameId currFrame, const FrameIdsT *const frameIds) {
        //typedef Cuboid::Vector3 Vector3;

        Vector3              *prevPos(nullptr);
        FrameId              prevFrameId(0);
        std::vector<Vector3> prevCorners;
        char                 name[512];
        for (auto const &frameAndState : cuboid.getStates()) {
            const FrameId &frameId = frameAndState.first;
//            const PoseLoc &state   = frameAndState.second;
            if (frameIds && (frameId < frameIds->front() || frameId > frameIds->back()))
                continue;
            auto transform = cuboid.getTransform4x4(frameId);
            transform.matrix().template block<3, 1>(0, 3) *= scale;

            sprintf(name, "cuboid%d_time%u", cuboidId, frameId);

            std::vector<Vector3> corners;
            if (frameId == currFrame)
                drawCuboid(vis, transform, Vector3(0., .8, 0.), name, &corners);
            else
                drawCuboid(vis, transform, color, name, &corners);

            if (prevPos) {
                sprintf(name, "%s_line_%u_%u", cuboid.getName().c_str(), prevFrameId, frameId);
                vis.addLine(*prevPos, transform.template block<3, 1>(0, 3), Vector3(1., 0., 0.), name, 2.);

                sprintf(name, "%s_cornerline_%u_%u_%d", cuboid.getName().c_str(), prevFrameId, frameId, 0);
                vis.addLine(prevCorners.at(0), corners.at(0), Vector3::Zero(), name, 1.);
            }
            else
                prevPos = new Vector3;

            *prevPos = transform.template block<3, 1>(0, 3);
            prevFrameId = frameId;
            prevCorners = corners;
        }

        if (prevPos)
            delete prevPos;
    } //...drawCuboidStates
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_VIS_HPP
