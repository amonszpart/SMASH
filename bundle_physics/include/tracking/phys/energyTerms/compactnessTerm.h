//
// Created by bontius on 03/04/16.
//

#ifndef TRACKVIDEO_PHYS_COMPACTNESSTERM_H
#define TRACKVIDEO_PHYS_COMPACTNESSTERM_H

#include "ceres/typedefs.h"

namespace ceres { class CostFunction; }

namespace tracking {
  namespace bundle_physics {
    struct CompactnessCostFunctor {
        public:
            enum { NUM_RESIDUALS = 1 };
            using CeresScalar = ceres::CeresScalar;

            template <typename _Scalar>
            CompactnessCostFunctor(_Scalar const* const sizePrior);

            virtual ~CompactnessCostFunctor() = default;

            template <typename T>
            bool operator()(T const p0[3], T* residuals ) const;

            template <typename _Scalar>
            static ceres::CostFunction* Create(_Scalar const* const sizePrior);
        protected:
            Eigen::Matrix<CeresScalar,3,1> _sizePrior;
            CeresScalar _maxPrior;
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    }; //...struct PointCostFunctor
  } //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_COMPACTNESSTERM_H
