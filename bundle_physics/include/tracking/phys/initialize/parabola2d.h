//
// Created by bontius on 30/04/16.
//

#ifndef TRACKVIDEO_PHYS_PARABOLA2D_H
#define TRACKVIDEO_PHYS_PARABOLA2D_H

#include "tracking/common/eigen.h"
#include "tracking/phys/initialize/circle.h"
#include <tracking/phys/typedefsGeometry.h>
#include <tracking/phys/typedefs.h>
#include <tracking/common/typedefs.h>
#include <tracking/common/trackFwDecl.h>
#include <tracking/common/track.h>

namespace tracking {
  namespace bundle_physics {

    struct Parabola2d {
        public:
            void addCircle(FrameId const frameId, Circle circle);
            bool hasCircle(FrameId const frameId) const { return _circles.find(frameId) != _circles.end(); }
            Circle const& getCircle(FrameId const frameId) const { return _circles.at(frameId); }
//            std::map<FrameId, Circle> const& getCircles() const { return _circles; }
            std::vector<Circle> getCircles(FrameId const frameId) const { if (hasCircle(frameId)) return {getCircle(frameId)}; else return {}; }
        public:
            Eigen::Vector4d       coeffs;
            std::vector <Scalar>  oX, oY;
            std::vector <FrameId> frameIds;
            Track2D const& getTrack() const { return _track; }
        protected:
            Track2D               _track;
            std::map<FrameId, Circle> _circles;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    typedef std::map <std::pair<CuboidId, PartId>, Parabola2d> Parabola2ds;
  }
}

#endif //TRACKVIDEO_PARABOLA2D_H
