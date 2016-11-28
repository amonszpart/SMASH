//
// Created by bontius on 06/01/16.
//

#ifndef TRACKVIDEO_CONSERVATIONTERMS_H
#define TRACKVIDEO_CONSERVATIONTERMS_H

#include "tracking/phys/physFunctorInfoFwDecl.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/util/exception.h"
#include "ceres/functorInfo.h"

namespace ceres { class Problem; }

namespace tracking {
  namespace bundle_physics {
    class PhysIndexer;
    class Weights;
    class Consts;


    DEFINE_EXCEPTION(BundleWithPhysics_AddConservationTerms_InfiniteMass)

    void addConservationTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        ceres::FunctorInfosT        & costFunctors,
        const FrameIdsT             & frameIds,
        const Weights               & weights,
        const CuboidsT              & cuboids,
        const std::vector<CuboidId> & participants,
        const Consts                & consts
    ); //...addConservationTerms

    void sfmAddConservationTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        PhysFunctorInfosT           & costFunctors,
        FrameIdsT             const & frameIds,
        Weights               const & weights,
        CuboidsT              const & cuboids,
        std::vector<CuboidId> const & participants,
        Consts                const & consts
    );

    void addConservationTerms(
        ceres::Problem              & problem,
        PhysIndexer                 & indexer,
        ceres::FunctorInfosT        & costFunctors,
        const FrameIdsT             & frameIds,
        const Weights               & weights,
        const Cuboid                & cuboid, // assumed to have id 0 in indexer
        const Consts                & consts
    );
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_CONSERVATIONTERMS_H
