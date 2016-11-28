//
// Created by bontius on 07/05/16.
//
#include "tracking/phys/energyTerms/impl/CoRBoundsTerm.hpp"
#include "tracking/phys/energyTerms/CoRBoundsTerm.h"
#include "ceres/functorInfo.h"

namespace tracking {
namespace bundle_physics {

template
void addCorFunctor(
    ceres::Problem             & problem,
    PhysIndexer                & indexer,
    ceres::FunctorInfosT       & costFunctors,
    FrameIdsT             const& frameIds,
    Weights               const& weights,
    Cuboid                const& cuboid0, // assumed to have id 0 in indexer
    Cuboid                const& cuboid1, // assumed to have id 1 in indexer
    Consts                const& consts);

template
void addCorFunctorInfMass(
    ceres::Problem             & problem,
    PhysIndexer                & indexer,
    ceres::FunctorInfosT       & costFunctors,
    FrameIdsT             const& frameIds,
    Weights               const& weights,
    Cuboid                const& cuboid0, // assumed to have id 0 in indexer
    Consts                const& consts);
} //...ns bundle_phsyics
} //...ns tracking
