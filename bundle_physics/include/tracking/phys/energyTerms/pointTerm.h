//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_POINTTERMS_H
#define TRACKVIDEO_POINTTERMS_H

#include "tracking/phys/physFunctorInfoFwDecl.h"
#include "tracking/phys/initialize/assignments.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "ceres/functorInfo.h"
#include <vector>

namespace ceres { class Problem; class LossFunctionWrapper; }
namespace tracking {
  class Mapper;

  namespace bundle_physics {
    class Weights;
    class Consts;
    class PhysIndexer;

    void addPointTerms(
        ceres::Problem                   &       problem,
        PhysIndexer                      &       indexer,
        ceres::FunctorInfosT             &       costFunctors,
        GroupedTracks2d             const&       tracks2d,
        GroupsCuboidsT              const&       groupsCuboids,
        CuboidsT                    const&       cuboids,
        Mapper                      const&       mapper,
        FrameIdsT                   const&       frameIds,
        Weights                     const&       weights,
        Consts                      const&       consts,
        ceres::LossFunctionWrapper       * const pointLoss
    );

    void addPointTerms(
        ceres::Problem                   &       problem,
        PhysIndexer                      &       indexer,
        PhysFunctorInfosT                &       costFunctors,
        TracksToCuboidsT            const&       assignments,
        GroupedTracks2d             const&       tracks2d,
        CuboidsT                    const&       cuboids,
        Mapper                      const&       mapper,
        FrameIdsT                   const&       frameIds,
        Weights                     const&       weights,
        Consts                      const&       consts
    );
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_POINTTERMS_H
