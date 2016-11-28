//
// Created by bontius on 11/01/16.
//

#ifndef TRACKVIDEO_GRAVITYDOWNTERM_HPP
#define TRACKVIDEO_GRAVITYDOWNTERM_HPP

#include "tracking/phys/weights.h"
#include "tracking/phys/consts.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/ceresUtil.h"
#include "ceres/functorInfo.h"
#include "ceres/ceresUtil.h"

namespace tracking {
  namespace bundle_physics {

    struct GravityCostFunctor {
        public:
            typedef ceres::CeresScalar CeresScalar; //!< \copydoc ceres::CeresScalar
            enum { NUM_RESIDUALS = 1 };

            GravityCostFunctor(const double sqrtWeight, const double fps, const double targetG = 9.81 )
                : _sqrtWeight(sqrtWeight), _fpsSqr( fps * fps ), _targetG( targetG )
            {}

            /**
             * \param[out] g [3 values] Gravity vector in world coordinates.
             * \note free should not matter here.
             */
            template <typename T>
            static void getWorldGravityFromFpsSqr(const T *const rotG, const T *const a, const double fpsSqr, T *g) {
                T g0[3];
                g0[0] = T(0.);
                g0[1] = _getGravityNorm(a, fpsSqr);
                g0[2] = T(0.);
                static T y0(0.);
                rotatePoint( &y0, rotG, g0, g );
            }

            /**
             * \param[out] g [3 values] Gravity vector in world coordinates.
             * \note free should not matter here.
             */
            template <typename T>
            static void getWorldGravity(const T *const rotG, const T *const a, const double fps, T *g) {
                getWorldGravityFromFpsSqr(rotG, a, fps * fps, g);
            }

            template <typename T, typename T2>
            static CeresScalar getAFromGravity( const T gravity[3], const T2 fps )
            {
                // g = T(2.) * a[0] * T(fpsSqr);
                // a = g / 2 / fps / fps
                return Eigen::Map<const Eigen::Matrix<T,3,1> >(gravity).template norm() / ( T(2.) * fps * fps );
            } //...getAFromGravity

            /**
             *
             * \tparam     T                Value type, usually ceres::Jet or double.
             *
             * \param[in]  angleAxisOffset  [6 values] AngleAxis description of the rotation \f$ \mathbf{R}_{parabola} \f$ of the parabola, and 3D translation \f$ \mathbf{c}_{parabola} \f$ of parabola.
             * \param[in]  abtScale         [3 values] Parabola shape parameters \f$ a, b, s \f$.
             */
            template <typename T>
            bool operator()( const T a[1], T* residuals ) const // a (t/tScale)^2 + b (t/tScale) = 0
            {
                // g = 2. * abs[0] * fps * fps
                residuals[0] = T(_sqrtWeight) * (T(_targetG) - _getGravityNorm(a, _fpsSqr));

                return true;
            } //...operator()

        [[deprecated]]
            static ceres::CostFunction* Create( const double sqrtWeight, const double fps, const double targetG )
            {
                return (new ceres::AutoDiffCostFunction<GravityCostFunctor, NUM_RESIDUALS, PhysIndexer::PARABOLA_SHARED_A_STRIDE >(
                    new GravityCostFunctor(sqrtWeight,fps,targetG)));
            }
        protected:
            template <typename T>
            static inline T _getGravityNorm(const T *const a, const double fpsSqr)
            {
                return T(2.) * a[0] * T(fpsSqr);
            }

            const double _sqrtWeight;
            const double _fpsSqr;
            const double _targetG;
    }; //...GravityCostFunctor

    struct GravityDownCostFunctor {
        public:
            typedef ceres::CeresScalar CeresScalar; //!< \copydoc ceres::CeresScalar
            enum { NUM_RESIDUALS = 1 };

            template <typename _Vector3>
            GravityDownCostFunctor(const CeresScalar sqrtWeight, const CeresScalar fps, const _Vector3& targetDown)
                : _sqrtWeight(sqrtWeight), _fpsSqr( fps * fps ), _targetDown( {targetDown(0),targetDown(1),targetDown(2)} )
            {}

            /**
             * \tparam     T                Value type, usually ceres::Jet or double.
             * \param[in]  angleAxisOffset  [6 values] AngleAxis description of the rotation \f$ \mathbf{R}_{parabola} \f$ of the parabola, and 3D translation \f$ \mathbf{c}_{parabola} \f$ of parabola.
             * \param[in]  abtScale         [3 values] Parabola shape parameters \f$ a, b, s \f$.
             */
            template <typename T>
            bool operator()( const T rotG[2], const T a[1], T* residuals ) const // a (t/tScale)^2 + b (t/tScale) = 0
            {
                // g = 2. * a * fps * fps
                T g[3];
                GravityCostFunctor::getWorldGravityFromFpsSqr(rotG, a, _fpsSqr, g);

                residuals[0] = T(_sqrtWeight) * ( T(9.81) - (g[0] * T(_targetDown[0]) + g[1] * T(_targetDown[1]) + g[2] * T(_targetDown[2])) );
                //std::cout << "g: " << ceres::printJetVector3( g ) << ", vs. down: " << _targetDown[0] << "," << _targetDown[1] << "," << _targetDown[2] << std::endl;

                return true;
            } //...operator()

            // Factory to hide the construction of the CostFunction object from
            // the client code.
            template <typename _Vector3>
            static ceres::CostFunction* Create( const CeresScalar sqrtWeight, const CeresScalar fps, const _Vector3& targetG )
            {
                return (new ceres::AutoDiffCostFunction<GravityDownCostFunctor, NUM_RESIDUALS, PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE, PhysIndexer::PARABOLA_SHARED_A_STRIDE >(
                    new GravityDownCostFunctor(sqrtWeight,fps,targetG)));
            }
        protected:

            const ceres::CeresScalar               _sqrtWeight;
            const ceres::CeresScalar               _fpsSqr;
            const std::array<ceres::CeresScalar,3> _targetDown;
    }; //...GravityCostFunctor

    void addGravityTerm( ceres::Problem& problem, PhysIndexer& indexer, ceres::FunctorInfosT& costFunctors, const Weights& weights, const Consts& consts );
    void initGravityTerm(ceres::Problem &problem, PhysIndexer &indexer, double const k_fps);

  } //...bundle_physics
} //...tracking

#endif //TRACKVIDEO_GRAVITYDOWNTERM_HPP
