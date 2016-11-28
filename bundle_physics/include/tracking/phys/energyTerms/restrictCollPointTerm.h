//
// Created by bontius on 10/01/16.
//

#ifndef TRACKVIDEO_RESTRICTCOLLPOINTTERM_H
#define TRACKVIDEO_RESTRICTCOLLPOINTTERM_H

#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/phys/typedefs.h"

namespace ceres { class Problem; }
namespace tracking {
  namespace bundle_physics {
    class Weights;
    class PhysIndexer;

    template<typename _FunctorInfos>
    void restrictCollPoint( CuboidsT const& cuboids, const CollId collId, const Weights &weights, ceres::Problem &problem, PhysIndexer &indexer, _FunctorInfos &costFunctors);
  } //...ns tracking
} //...ns bundle_physics

#endif //TRACKVIDEO_RESTRICTCOLLPOINTTERM_H
