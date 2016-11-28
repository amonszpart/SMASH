//
// Created by bontius on 27/12/15.
//

#ifndef TRACKVIDEO_PHYS_IO_PARABOLA_H
#define TRACKVIDEO_PHYS_IO_PARABOLA_H

#include "tracking/common/eigen.h"
#include "tracking/common/util/exception.h" // DEFINE_EXCEPTION
#include "picojson/picojson.h"
#include <string>

namespace tracking {
  namespace bundle_physics {
    class BundleWithPhysicsResult;
    class Parabola;

    namespace io {
      DEFINE_EXCEPTION(WriteInitialization_GravityScaleDiscrepancy)
      DEFINE_EXCEPTION(WriteInitialization_ASingleCollisionExpected)
      DEFINE_EXCEPTION(WriteInitialization_PoseDiscrepancy)

      int  writeInitialization(const BundleWithPhysicsResult &result, const std::string &path, const double fps);

      bool parseShared(const picojson::object &object, BundleWithPhysicsResult &res);

      bool parseParabola(const picojson::object &object, Parabola &parabola, Eigen::Vector3d &momentum);

      DEFINE_EXCEPTION(ParseParabolas_DuplicatePartId)
      DEFINE_EXCEPTION(ParseParabolas_NoCuboidId)
      DEFINE_EXCEPTION(ParseParabolas_DuplicateCuboidId)

      bool parseParabolas(const picojson::object &object, BundleWithPhysicsResult &res);

      DEFINE_EXCEPTION(ReadInitialization_CantParseObject)

      int readInitialization(BundleWithPhysicsResult &result, const std::string &inPath);
    }
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_IO_PARABOLA_H
