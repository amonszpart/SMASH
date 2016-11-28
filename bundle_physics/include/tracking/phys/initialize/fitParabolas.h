//
// Created by bontius on 05/04/16.
//

#ifndef TRACKVIDEO_PHYS_FITPARABOLA_H
#define TRACKVIDEO_PHYS_FITPARABOLA_H

#include "tracking/phys/initialize/parabola2d.h"
#include "tracking/phys/initialize/assignments.h"
#include "tracking/phys/typedefs.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include <opencv2/core/types.hpp>
#include <opencv2/core/mat.hpp>

namespace tracking {
class Mapper;
namespace bundle_physics {
class Weights;

class BundleWithPhysicsResult;

BundleWithPhysicsResult
fit2dParabolas(GroupedTracks2d const& tracks2d, TracksToCuboidsT const& assignments, CuboidsT const& cuboids,
               RgbsT const& rgbs, FrameIdsT const& frameIds, Mapper const& mapper, Weights const& weights);

BundleWithPhysicsResult fitParabola(
    Parabola2ds const& parabola2ds, FrameIdsT const& frameIds, Mapper const& mapper, Weights const& weights,
    CuboidsT const& cuboids, RgbsT const& rgbs, int const show, double* const cost = nullptr,
    Scalar const collTimeArg = -1., float const flatLimit = 0.f, bool const saveRawCentroids = false);

BundleWithPhysicsResult
ransacParabolas(FrameIdsT const& frameIds, Mapper const& mapper, Weights const& weights, CuboidsT const& cuboids,
                RgbsT const& rgbs, bool const debug, cv::Size const& bbox, std::array<int, 4>* const crops,
                int const erodeIterations, int const closeIterations, int const morphSize, int const minBlobArea,
                cv::Mat* const bg, int const ignoreAroundCollTime, int const lookAround, float const halfSpreadPos,
                float const flatLimit, std::string const maskPath, double learningRate, bool const saveRawCentroids,
                float const inlierDiv);
} //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_FITPARABOLA_H
