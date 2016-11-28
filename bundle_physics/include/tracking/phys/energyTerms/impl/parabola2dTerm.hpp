//
// Created by bontius on 12/05/16.
//
//
#ifndef TRACKVIDEO_PHYS_PARABOLA2DTERM_HPP
#define TRACKVIDEO_PHYS_PARABOLA2DTERM_HPP

#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/energyTerms/parabola2dTerm.h"

#include "tracking/phys/physIndexer.h"
#include "tracking/phys/ceresUtil.h"   // rotatePoint
#include "tracking/common/mapper.h"
#include "tracking/common/typedefs.h"
#include "ceres/typedefs.h"

namespace tracking {
  namespace bundle_physics {
    template<typename _Vector3>
    Parabola2DFunctor::Parabola2DFunctor(double const sqrtWeight, FrameId const& time, _Vector3 const& observedPos, Mapper const& mapper)
        : _sqrtWeight(sqrtWeight), _time(time), _timeSqr(time * time),
          _observedPos({observedPos(0),observedPos(1),observedPos(2)}),
          _observedUv((observedPos(2) < 0.f) ? decltype(_observedUv){(observedPos(0) - mapper.getCx()) / mapper.getFx(),
                                                                     (observedPos(1) - mapper.getCy()) / mapper.getFy()}
                                             : decltype(_observedUv){ observedPos(0)/observedPos(2),
                                                                      observedPos(1)/observedPos(2)})
//    : _fx( intr(0,0) ),
//        _fy( intr(1,1) ),
//        _u((u - intr(0,2))/_fx),
//        _v((v - intr(1,2))/_fy),
    {
        if (std::isnan(_observedUv[0]) || std::isnan(_observedUv[1])) {
            std::cerr << "[" << __func__ << "] " << "NAN: " << _observedUv[0] << "," << _observedUv[1]<< std::endl;
            throw new std::runtime_error("");
        }
    }

    /**
     * \param[in]  rotG        The two later rotation angles of corresponding to the gravity vector ({\f$ \beta_{x},\beta_{y1} \f$} from {\f$ \beta_{y0}, \beta_{x}, \beta_{y1} \f$}).
     * \param[in]  a           Squared parameter of parabola (gravity scale), aka \f$ b_1 \f$.
     * \param[in]  translation 3D translation of parabola (position of object at collision time), aka \f$ b_4 \f$.
     * \param[in]  free        Free parameters of parabola. Angle "y0" (\f$\beta_{y0}\f$) in degrees, linear term "b" (\f$b_2\f$), and x scaling "s" (\f$b_3\f$).
     * \param[in]  collTime    Time of collision, \f$ t^c \f$.
     */
    template<typename T>
    bool Parabola2DFunctor::operator()(const T rotG[2], const T a[1], const T translation[3], const T *const free, const T collTime[1], T *residuals) const {
        T x[3];
        ParabolaCostFunctor::getPositionAtTime(x, rotG, a, translation, free, T(_time) - collTime[0]);
        if (x[2] == T(0.))
            return false;
        T invDepth = T(1.) / x[2];
        residuals[0] = T(_sqrtWeight) * (x[0] * invDepth - T(_observedUv[0]));
        residuals[1] = T(_sqrtWeight) * (x[1] * invDepth - T(_observedUv[1]));
//        residuals[2] = T(_sqrtWeight * 0.033) * (x[2] - T(_observedPos[2]));

        return true;
    } //...operator()

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    template<typename _Vector3>
    ceres::CostFunction* Parabola2DFunctor::Create(const double weight, const FrameId time, const _Vector3 &observedPos, Mapper const& mapper) {
        return (new ceres::AutoDiffCostFunction<Parabola2DFunctor, NUM_RESIDUALS,
            PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE, PhysIndexer::PARABOLA_SHARED_A_STRIDE, PhysIndexer::PARABOLA_TRANSLATION_STRIDE, PhysIndexer::PARABOLA_FREE_STRIDE, PhysIndexer::COLL_TIME_STRIDE>(
            new Parabola2DFunctor(weight, time, observedPos, mapper)));
    } //...Create()


    template<typename _Vector3>
    DepthPriorFunctor::DepthPriorFunctor(double const sqrtWeight, FrameId const& time, _Vector3 const& observedPos, Mapper const& mapper)
        : _sqrtWeight(sqrtWeight), _time(time), _timeSqr(time * time),
        _observedPos({observedPos(0),observedPos(1),observedPos(2)}),
        _observedUv((observedPos(2) < 0.f) ? decltype(_observedUv){(observedPos(0) - mapper.getCx()) / mapper.getFx(),
                                                                   (observedPos(1) - mapper.getCy()) / mapper.getFy()}
                                           : decltype(_observedUv){ observedPos(0)/observedPos(2),
                                                                    observedPos(1)/observedPos(2)})
//    : _fx( intr(0,0) ),
//        _fy( intr(1,1) ),
//        _u((u - intr(0,2))/_fx),
//        _v((v - intr(1,2))/_fy),
    {
        if (std::isnan(_observedUv[0]) || std::isnan(_observedUv[1])) {
            std::cerr << "[" << __func__ << "] " << "NAN: " << _observedUv[0] << "," << _observedUv[1]<< std::endl;
            throw new std::runtime_error("");
        }
    }

    /**
     * \param[in]  rotG        The two later rotation angles of corresponding to the gravity vector ({\f$ \beta_{x},\beta_{y1} \f$} from {\f$ \beta_{y0}, \beta_{x}, \beta_{y1} \f$}).
     * \param[in]  a           Squared parameter of parabola (gravity scale), aka \f$ b_1 \f$.
     * \param[in]  translation 3D translation of parabola (position of object at collision time), aka \f$ b_4 \f$.
     * \param[in]  free        Free parameters of parabola. Angle "y0" (\f$\beta_{y0}\f$) in degrees, linear term "b" (\f$b_2\f$), and x scaling "s" (\f$b_3\f$).
     * \param[in]  collTime    Time of collision, \f$ t^c \f$.
     */
    template<typename T>
    bool DepthPriorFunctor::operator()(const T rotG[2], const T a[1], const T translation[3], const T *const free, const T collTime[1], T *residuals) const {
        T x[3];
        ParabolaCostFunctor::getPositionAtTime(x, rotG, a, translation, free, T(_time) - collTime[0]);
        if (x[2] == T(0.))
            return false;
        residuals[0] = T(_sqrtWeight) * (x[2] - T(_observedPos[2]));

        return true;
    } //...operator()

    // Factory to hide the construction of the CostFunction object from
    // the client code.
    template<typename _Vector3>
    ceres::CostFunction* DepthPriorFunctor::Create(const double weight, const FrameId time, const _Vector3 &observedPos, Mapper const& mapper) {

        return (new ceres::AutoDiffCostFunction<DepthPriorFunctor, NUM_RESIDUALS,
            PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE, PhysIndexer::PARABOLA_SHARED_A_STRIDE, PhysIndexer::PARABOLA_TRANSLATION_STRIDE, PhysIndexer::PARABOLA_FREE_STRIDE, PhysIndexer::COLL_TIME_STRIDE>(
            new DepthPriorFunctor(weight, time, observedPos, mapper)));
    } //...Create()
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_PARABOLA2DTERM_HPP
