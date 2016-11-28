//
// Created by bontius on 10/01/16.
//

#ifndef TRACKVIDEO_VISPHYS_H
#define TRACKVIDEO_VISPHYS_H

#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/phys/typedefsGeometry.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/tracksFwDecl.h"

namespace tracking {
  class Mapper;

  namespace bundle_physics {
    class Consts;
    class PhysIndexer;
    class BundleWithPhysicsResult;

    void showPhysOutput(BundleWithPhysicsResult const& out, const CuboidsT &cuboids, const PhysIndexer &indexer, const Consts &consts, std::string const showFlags, FrameIdsT const* const frameIds = nullptr);

    void createMovie(CuboidsT const& cuboids, const tracking::LinRgbsT& rgbs, const FrameIdsT& frameIds, const Mapper& mapper, const CuboidsT* const inCuboids = nullptr
                      , const Tracks3D *const tracks3d = nullptr, const Tracks2D* const tracks2d = nullptr, const Vector3* const gravity = nullptr, int const fps = 60);
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_VISPHYS_H_H
