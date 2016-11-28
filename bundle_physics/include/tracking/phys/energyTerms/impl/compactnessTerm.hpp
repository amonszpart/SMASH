//
// Created by bontius on 03/04/16.
//

#ifndef TRACKVIDEO_PHYS_COMPACTNESSTERM_HPP
#define TRACKVIDEO_PHYS_COMPACTNESSTERM_HPP

#include "tracking/phys/physIndexer.h"
#include "ceres/typedefs.h"
#include "ceres/cost_function.h"
#include "ceres/ceres.h"

namespace tracking {
  namespace bundle_physics {

    template <typename _Scalar>
    CompactnessCostFunctor::CompactnessCostFunctor(_Scalar const* const sizePrior)
        : _sizePrior(sizePrior[0],sizePrior[1],sizePrior[2]),
        _maxPrior(std::max(std::max(_sizePrior(0),_sizePrior(1)),_sizePrior(2)) / 2.)
    {}

    template <typename T>
    bool CompactnessCostFunctor::operator()(T const p0[3], T* residuals ) const {
        //residuals[0] = T(_maxPrior) - ceres::abs(p0[0]);
        //residuals[1] = T(_maxPrior) - ceres::abs(p0[1]);
        //residuals[2] = T(_maxPrior) - ceres::abs(p0[2]);

        T const len = ceres::length(p0);
        residuals[0] = std::max(T(_maxPrior), len);

        return true;
    } //...operator()

    template <typename _Scalar>
    ceres::CostFunction* CompactnessCostFunctor::Create(_Scalar const* const sizePrior) {
        return new ceres::AutoDiffCostFunction<CompactnessCostFunctor,NUM_RESIDUALS,PhysIndexer::POINTS_STRIDE>( new CompactnessCostFunctor(sizePrior) );
    }
  } //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_COMPACTNESSTERM_HPP
