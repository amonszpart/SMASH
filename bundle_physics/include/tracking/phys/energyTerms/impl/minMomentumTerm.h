//
// Created by bontius on 07/01/16.
//

#ifndef TRACKVIDEO_MINMOMENTUMTERM_H
#define TRACKVIDEO_MINMOMENTUMTERM_H

#include "ceres/ceres.h"
#include "tracking/phys/physIndexer.h"


namespace tracking {
namespace bundle_physics {

struct MinMomentumFunctor
{
        enum { NUM_RESIDUALS = 3 };

        MinMomentumFunctor( const double weight )
            : _weight( weight )
        {}

        template <typename T>
        bool operator()( const T* const momentum, T* residuals) const
        {
            residuals[0] = T(_weight) * (momentum[0]);
            residuals[1] = T(_weight) * (momentum[1]);
            residuals[2] = T(_weight) * (momentum[2]);

            return true;
        } //...operator()

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double weight) {
            return (new ceres::AutoDiffCostFunction<MinMomentumFunctor, NUM_RESIDUALS, PhysIndexer::MOMENTUM_STRIDE >(
                new MinMomentumFunctor(weight)));
        }
    protected:
        const double  _weight;
}; //...MinMomentumFunctor

}
}

#endif //TRACKVIDEO_MINMOMENTUMTERM_H
