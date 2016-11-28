//
// Created by bontius on 12/05/16.
//

#ifndef TRACKVIDEO_PHYS_DATATERM_H
#define TRACKVIDEO_PHYS_DATATERM_H


#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/typedefs.h"
#include "ceres/functorInfo.h"
#include "ceres/typedefs.h"

namespace ceres { class CostFunction; class Problem; }
namespace tracking {
  class Mapper;
  namespace bundle_physics {
    class PhysIndexer;

    struct Parabola2DFunctor {
        public:
            enum { NUM_RESIDUALS = 2 };
            using CeresScalar = ceres::CeresScalar;

            template<typename _Vector3>
            Parabola2DFunctor(double const sqrtWeight, FrameId const& time, _Vector3 const& observedPos, Mapper const& mapper);

            /**
             * \param[in]  rotG        The two later rotation angles of corresponding to the gravity vector ({\f$ \beta_{x},\beta_{y1} \f$} from {\f$ \beta_{y0}, \beta_{x}, \beta_{y1} \f$}).
             * \param[in]  a           Squared parameter of parabola (gravity scale), aka \f$ b_1 \f$.
             * \param[in]  translation 3D translation of parabola (position of object at collision time), aka \f$ b_4 \f$.
             * \param[in]  free        Free parameters of parabola. Angle "y0" (\f$\beta_{y0}\f$) in degrees, linear term "b" (\f$b_2\f$), and x scaling "s" (\f$b_3\f$).
             * \param[in]  collTime    Time of collision, \f$ t^c \f$.
             */
            template<typename T>
            bool operator()(const T rotG[2], const T a[1], const T translation[3], const T *const free, const T collTime[1], T *residuals) const;

            // Factory to hide the construction of the CostFunction object from
            // the client code.
            template<typename _Vector3>
            static ceres::CostFunction *Create(const double weight, const FrameId time, const _Vector3 &observedPos, Mapper const& mapper);

        protected:
            CeresScalar                const _sqrtWeight;
            FrameId                    const _time, _timeSqr;
            std::array<CeresScalar, 3> const _observedPos;
            std::array<CeresScalar, 2> const _observedUv;
    }; //...struct ParabolaCostFunctor

    struct DepthPriorFunctor {
        public:
            enum { NUM_RESIDUALS = 1 };
            using CeresScalar = ceres::CeresScalar;

            template<typename _Vector3>
            DepthPriorFunctor(double const sqrtWeight, FrameId const& time, _Vector3 const& observedPos, Mapper const& mapper);

            /**
             * \param[in]  rotG        The two later rotation angles of corresponding to the gravity vector ({\f$ \beta_{x},\beta_{y1} \f$} from {\f$ \beta_{y0}, \beta_{x}, \beta_{y1} \f$}).
             * \param[in]  a           Squared parameter of parabola (gravity scale), aka \f$ b_1 \f$.
             * \param[in]  translation 3D translation of parabola (position of object at collision time), aka \f$ b_4 \f$.
             * \param[in]  free        Free parameters of parabola. Angle "y0" (\f$\beta_{y0}\f$) in degrees, linear term "b" (\f$b_2\f$), and x scaling "s" (\f$b_3\f$).
             * \param[in]  collTime    Time of collision, \f$ t^c \f$.
             */
            template<typename T>
            bool operator()(const T rotG[2], const T a[1], const T translation[3], const T *const free, const T collTime[1], T *residuals) const;

            // Factory to hide the construction of the CostFunction object from
            // the client code.
            template<typename _Vector3>
            static ceres::CostFunction *Create(const double weight, const FrameId time, const _Vector3 &observedPos, Mapper const& mapper);

        protected:
            CeresScalar                const _sqrtWeight;
            FrameId                    const _time, _timeSqr;
            std::array<CeresScalar, 3> const _observedPos;
            std::array<CeresScalar, 2> const _observedUv;
    }; //...struct ParabolaCostFunctor

    class Weights;
    class Consts;
    class BundleWithPhysicsResult;
#if 0
    void addParabolaTerms(
        ceres::Problem                & problem,
        PhysIndexer                   & indexer,
        ceres::FunctorInfosT          & costFunctors,
        FrameIdsT                const& frameIds,
        Weights                  const& weights,
        CuboidsT                 const& cuboids,
        Consts                   const& consts,
        BundleWithPhysicsResult  const* const initial
    );
#endif
  } //...ns bundle_physics
} //...ns tracking
#endif //TRACKVIDEO_PHYS_DATATERM_H
