#ifndef TRACKING_PHYS_INITIALIZE_INITIALIZE_H
#define TRACKING_PHYS_INITIALIZE_INITIALIZE_H

#include "tracking/phys/physFunctorInfoFwDecl.h"
#include "tracking/phys/typedefs.h"
#include "tracking/common/typedefs.h"

namespace ceres { class Problem; }
namespace tracking {
  namespace bundle_physics {
    class BundleWithPhysicsResult;
    class Consts;
    class Weights;
    class PhysIndexer;

    void initCollPoint2(
        ceres::Problem                &      problem,
        PhysIndexer                   &      indexer,
        const Weights                 &      weights,
        const std::vector<CuboidId>   &      participants,
        const Consts                  &      consts,
        const BundleWithPhysicsResult *const initial);

    void initialize(
        ceres::Problem               &       problem,
        PhysIndexer                  &       indexer,
        PhysFunctorInfosT       const&       costFunctors,
        FrameIdsT               const&       frameIds,
        GroupedTracks2d         const&       tracks2d,
        TracksToCuboidsT        const&       assignments,
        Weights                 const&       weights,
        Consts                  const&       consts,
        Mapper                  const&       mapper,
        int                     const        nCuboids,
        std::vector<CuboidId>   const&       participants,
        CollId                  const        collId,
        BundleWithPhysicsResult const* const initial);

  } //...ns bundle_physics
}//...ns tracking

#endif // TRACKING_PHYS_INITIALIZE_INITIALIZE_H