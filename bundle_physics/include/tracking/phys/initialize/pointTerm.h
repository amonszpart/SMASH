//
// Created by bontius on 02/04/16.
//

#ifndef TRACKVIDEO_POINTTERM_H
#define TRACKVIDEO_POINTTERM_H

#include "tracking/phys/physFunctorInfoFwDecl.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/phys/initialize/assignments.h"

namespace ceres { class Problem; class LossFunctionWrapper; }

namespace tracking {
  class Mapper;
  namespace bundle_physics {
    class PhysIndexer;
    class Consts;
    class Weights;

    void addBatchPointTerms(
        ceres::Problem         &problem,
        PhysIndexer            &indexer,
        PhysFunctorInfosT      &costFunctors,
        TracksToCuboidsT const &assignments,
        GroupedTracks2d  const &tracks2d,
        CuboidsT         const &cuboids,
        Mapper           const &mapper,
        FrameIdsT        const &frameIds,
        Weights          const &weights,
        Consts           const &consts,
        ceres::LossFunctionWrapper *const pointLoss
    );
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_POINTTERM_H
