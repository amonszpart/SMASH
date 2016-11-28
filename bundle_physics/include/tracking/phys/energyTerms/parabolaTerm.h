//
// Created by bontius on 02/05/16.
//

#ifndef TRACKVIDEO_PHYS_PARABOLATERM_H
#define TRACKVIDEO_PHYS_PARABOLATERM_H

#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/common/typedefs.h"
#include "ceres/functorInfo.h"
#include "ceres/typedefs.h"

namespace ceres { class CostFunction; class Problem; }
namespace tracking {
  class Mapper;
  namespace bundle_physics {
    class PhysIndexer;

    struct ParabolaCostFunctor {
        public:
            enum { NUM_RESIDUALS = 3 };
            using CeresScalar = ceres::CeresScalar;

            template<typename _Vector3>
            ParabolaCostFunctor(double const sqrtWeight, FrameId const& time, _Vector3 const& observedPos);

            /** \brief Gives a point on a parabola at time \p time.
             * The parabola in space is represented as \f$ \begin{bmatrix} x \\ y \\ z \end{bmatrix}_t = \mathbf{R}_{parabola} * \begin{bmatrix} s~t \\ a~t^2 + b~t \\ 0 \end{bmatrix} + \mathbf{c}_{parabola} \f$
             * \tparam     T                Value type, usually ceres::Jet or double.
             * \param[out] x                [3 values] Output 3D point: \f$\begin{bmatrix} x,y,z \end{bmatrix}^T \f$.
             * \param[in]  shared           [3 values] 2 rotation angles \f$ \theta_x, \theta_{y1} \f$ and the parabola's \f$ a \f$ coefficient.
             * \param[in]  free             [6 values] 1 rotation angle \f$ \theta_{y0} \f$, translation \f$ c_{parabola,x},c_{parabola,y},c_{parabola,z} \f$
             *                              and linear parabola parameters \f$ b, s \f$.
             * \param[in]  time             Time of query point.
             */
            template<typename T>
            static void getPositionAtTime(T x[3], T const rotG[2], T const a[1], T const translation[3], T const* const free, T const* time);

            template<typename T>
            static void getPositionAtTime(T x[3], T const rotG[2], T const a[1], T const translation[3], T const* const free, T const& time);

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
            static ceres::CostFunction *Create(const double weight, const FrameId time, const _Vector3 &observedPos);

        protected:
            CeresScalar                const _sqrtWeight;
            FrameId                    const _time, _timeSqr;
            std::array<CeresScalar, 3> const _observedPos;
    }; //...struct ParabolaCostFunctor

    class Weights;
    class Consts;
    class BundleWithPhysicsResult;

    void addParabolaTerms(
        ceres::Problem               &       problem,
        PhysIndexer                  &       indexer,
        ceres::FunctorInfosT         &       costFunctors,
        FrameIdsT               const&       frameIds,
        Weights                 const&       weights,
        Mapper                  const&       mapper,
        CuboidsT                const&       cuboids,
        Consts                  const&       consts,
        BundleWithPhysicsResult const* const initial,
        bool                    const        use2dTerm = false
    );
  } //...ns bundle_physics
} //...ns tracking


//#include "tracking/phys/energyTerms/impl/parabolaTerm.hpp"

#endif //TRACKVIDEO_PHYS_PARABOLACOSTFUNCTOR_HPP
