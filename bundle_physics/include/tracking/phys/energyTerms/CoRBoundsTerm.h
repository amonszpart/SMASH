//
// Created by bontius on 02/05/16.
//

#ifndef TRACKVIDEO_PHYS_CORFUNCTOR_H
#define TRACKVIDEO_PHYS_CORFUNCTOR_H

//#include "ceres/functorInfo.h"
//#include "tracking/
#include "tracking/phys/typedefs.h"
#include "tracking/common/typedefs.h"
#include "ceres/typedefs.h"

namespace ceres { class Problem; class CostFunction; }

namespace tracking {
namespace bundle_physics {
class Cuboid;
class PhysIndexer;
class Consts;
class Weights;

/** \brief Enforces CoR to be in bounds 0..1 */
struct CoRCostFunctor {
    public:
        using CeresScalar = ceres::CeresScalar;
        enum { NUM_RESIDUALS = 2 };

        /**
         *  \param[in] sqrtWeight   Sqrt of term weight.
         *  \param[in] dt           Timestep size.
         */
        template <typename _Scalar>
        CoRCostFunctor(
            CeresScalar    const sqrtCoRWeight,
            CeresScalar    const sqrtKEWeight,
            CeresScalar    const dt,
            CeresScalar    const startFrame,
            _Scalar const* const sizeA,
            _Scalar const* const sizeB,
            SHAPE          const shapeA,
            SHAPE          const shapeB);

        /** \brief Estimates CoR: c = ((v'_b - v'_a) . n) / ((v_a - v_b) . n). */
        template <typename T>
        bool operator()(T const* const* unknowns, T* residuals) const;

        // Factory to hide the construction of the CostFunction object from the client code.
        template <typename _Scalar>
        static ceres::CostFunction *Create(
            CeresScalar    const sqrtCoRWeight,
            CeresScalar    const sqrtKEWeight,
            CeresScalar    const dt,
            CeresScalar    const startFrame,
            _Scalar const* const sizeA,
            _Scalar const* const sizeB,
            SHAPE          const shapeA,
            SHAPE          const shapeB);

    protected:
        CeresScalar                const _sqrtCoRWeight;
        CeresScalar                const _sqrtKEWeight;
        CeresScalar                const _dt;               //!< Time between frames (1/fps).
        CeresScalar                const _fps;              //!< Frames per sec (1/dt).
        CeresScalar                const _startFrame;
        std::array<CeresScalar, 3> const _sqrSizeA, _sqrSizeB; //!< Size of the object, used for inertia tensor estimation
        SHAPE                      const _shapeA, _shapeB;
}; //...CoRCostFunctor

template <typename _FunctorInfosT>
void addCorFunctor(
    ceres::Problem             & problem,
    PhysIndexer                & indexer,
    _FunctorInfosT             & costFunctors,
    FrameIdsT             const& frameIds,
    Weights               const& weights,
    Cuboid                const& cuboid0, // assumed to have id 0 in indexer
    Cuboid                const& cuboid1, // assumed to have id 1 in indexer
    Consts                const& consts);

template <typename _FunctorInfosT>
void addCorFunctorInfMass(
    ceres::Problem             & problem,
    PhysIndexer                & indexer,
    _FunctorInfosT             & costFunctors,
    FrameIdsT             const& frameIds,
    Weights               const& weights,
    Cuboid                const& cuboid0, // assumed to have id 0 in indexer
    Consts                const& consts);
} //...ns bundle_phsyics
} //...ns tracking

//#include "tracking/phys/energyTerms/impl/CoRBoundsTerm.hpp"

#endif //TRACKVIDEO_PHYS_CORFUNCTOR_H
