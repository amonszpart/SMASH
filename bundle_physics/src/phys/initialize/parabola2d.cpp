//
// Created by bontius on 09/05/16.
//

#include "tracking/phys/initialize/parabola2d.h"
#include "tracking/phys/initialize/circle.h"

namespace tracking {
  namespace bundle_physics {
    void Parabola2d::addCircle(FrameId const frameId, Circle circle) {
        if (!_track.hasPoint(frameId))
            _track.addPoint(frameId, {circle.getCentroid().x, circle.getCentroid().y});
        if (_circles.find(frameId) != _circles.end()) {
            std::cerr << "[" << __func__ << "] overwriting circle at frameId " << frameId << std::endl;
        }
        _circles.emplace(frameId,std::move(circle));
    }
  } //...ns bundle_phsyics
} //...ns tracking