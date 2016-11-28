//
// Created by bontius on 03/04/16.
//

#ifndef TRACKVIDEO_PHYS_PARABOLACOSTFUNCTOR_HPP
#define TRACKVIDEO_PHYS_PARABOLACOSTFUNCTOR_HPP

#include "tracking/phys/energyTerms/parabolaTerm.h"

#include "tracking/phys/physIndexer.h"
#include "tracking/phys/ceresUtil.h"   // rotatePoint
#include "tracking/common/typedefs.h"
#include "ceres/typedefs.h"

namespace tracking {
namespace bundle_physics {

template<typename _Vector3>
ParabolaCostFunctor::ParabolaCostFunctor(double const sqrtWeight, FrameId const& time, _Vector3 const& observedPos)
    : _sqrtWeight(sqrtWeight), _time(time), _timeSqr(time * time), _observedPos({observedPos(0), observedPos(1), observedPos(2)})
{}

/** \brief Gives a point on a parabola at time \p time.
 *
 *   The parabola in space is represented as
 *   \f$ \begin{bmatrix} x \\ y \\ z \end{bmatrix}_t =
 *   \mathbf{R}_{parabola} * \begin{bmatrix} s~t \\ a~t^2 + b~t \\ 0 \end{bmatrix} + \mathbf{c}_{parabola} \f$
 *
 * \tparam     T                Value type, usually ceres::Jet or double.
 * \param[out] x                [3 values] Output 3D point: \f$\begin{bmatrix} x,y,z \end{bmatrix}^T \f$.
 * \param[in]  shared           [3 values] 2 rotation angles \f$ \theta_x, \theta_{y1} \f$ and
 *                              the parabola's \f$ a \f$ coefficient.
 * \param[in]  free             [6 values] 1 rotation angle \f$ \theta_{y0} \f$,
 *                              translation \f$ c_{parabola,x},c_{parabola,y},c_{parabola,z} \f$
 *                              and linear parabola parameters \f$ b, s \f$.
 * \param[in]  time             Time of query point.
 */
template<typename T>
void ParabolaCostFunctor::getPositionAtTime(T x[3], T const rotG[2], T const a[1], T const translation[3],
                                            T const* const free, T const* time) {
    T p[3] = { free[PhysIndexer::PARABOLA_FREE_S_OFFSET] * time[0],                    // s * t
               (a[0] * time[0] + free[PhysIndexer::PARABOLA_FREE_B_OFFSET]) * time[0], // a * t^2 + b * t
               T(0.) };
    rotatePoint(free + PhysIndexer::PARABOLA_FREE_ANGLE_OFFSET, rotG, p, x);
    x[0] += translation[0];
    x[1] += translation[1];
    x[2] += translation[2];
}

template<typename T>
void ParabolaCostFunctor::getPositionAtTime(T x[3], T const rotG[2], T const a[1], T const translation[3],
                                            T const* const free, T const& time) {
    getPositionAtTime(x, rotG, a, translation, free, &time);
}

/**
 * \param[in]  rotG        The two later rotation angles of corresponding to the gravity vector
 *                         ({\f$ \beta_{x},\beta_{y1} \f$} from {\f$ \beta_{y0}, \beta_{x}, \beta_{y1} \f$}).
 * \param[in]  a           Squared parameter of parabola (gravity scale), aka \f$ b_1 \f$.
 * \param[in]  translation 3D translation of parabola (position of object at collision time), aka \f$ b_4 \f$.
 * \param[in]  free        Free parameters of parabola. Angle "y0" (\f$\beta_{y0}\f$) in degrees,
 *                         linear term "b" (\f$b_2\f$), and x scaling "s" (\f$b_3\f$).
 * \param[in]  collTime    Time of collision, \f$ t^c \f$.
 */
template<typename T>
bool ParabolaCostFunctor::operator()(const T rotG[2], const T a[1], const T translation[3], const T* const free,
                                     const T collTime[1], T* residuals) const {
    T x[3];
    getPositionAtTime(x, rotG, a, translation, free, T(_time) - collTime[0]);

    residuals[0] = T(_sqrtWeight) * (T(_observedPos[0]) - x[0]);
    residuals[1] = T(_sqrtWeight) * (T(_observedPos[1]) - x[1]);
    residuals[2] = T(_sqrtWeight * 0.33) * (T(_observedPos[2]) - x[2]);
    return true;
} //...operator()

// Factory to hide the construction of the CostFunction object from
// the client code.
template<typename _Vector3>
ceres::CostFunction* ParabolaCostFunctor::Create(const double weight, const FrameId time, const _Vector3 &observedPos) {
    return (new ceres::AutoDiffCostFunction<ParabolaCostFunctor, NUM_RESIDUALS,
        PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE, PhysIndexer::PARABOLA_SHARED_A_STRIDE,
        PhysIndexer::PARABOLA_TRANSLATION_STRIDE, PhysIndexer::PARABOLA_FREE_STRIDE, PhysIndexer::COLL_TIME_STRIDE>(
        new ParabolaCostFunctor(weight, time, observedPos)));
} //...Create()

} //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_PARABOLACOSTFUNCTOR_HPP
