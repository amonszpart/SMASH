//
// Created by bontius on 01/04/16.
//

#ifndef TRACKVIDEO_PHYS_BATCHPOINTTERM_HPP
#define TRACKVIDEO_PHYS_BATCHPOINTTERM_HPP

#include "parabolaTerm.hpp"
#include "poseTerm.hpp"
#include "tracking/phys/physIndexer.h"
#include <list>

namespace tracking {
namespace bundle_physics {

struct BatchPointCostFunctor {
    public:
        enum { NUM_RESIDUALS = 2 };
        using CeresScalar = ceres::CeresScalar;

        /**
         * \param[in] halfBBox Half size of bounding box centered around origin, constraining the object size (points' distance from origin).
         */
        template <typename _Matrix3, typename _Scalar>
        BatchPointCostFunctor(
            _Matrix3    const&       intr,
            CeresScalar const        time,
            CeresScalar const        dt,
            _Scalar     const* const size,
            int         const        collTimeLowerBound,
            int         const        substepsPerFrame,
            SHAPE       const        shape,
            std::list< Eigen::Vector2d > const& observations
        )
            : _fx( intr(0,0) ),
              _fy( intr(1,1) ),
              _observationTime(time ),
              _dtHalf( dt / 2. ),
              _sqrSize{size[0] * size[0], size[1] * size[1], size[2] * size[2]},
              _integrationSteps(substepsPerFrame * std::max(1.,std::abs(collTimeLowerBound - _observationTime))),
              _shape( shape )
        {
            // _u((u - intr(0,2))/_fx),
            // _v((v - intr(1,2))/_fy),
            _observations.reserve(observations.size());
            for ( Eigen::Vector2d const& obs : observations )
                _observations.push_back({obs(0)-intr(0,2)/_fx,obs(1)-intr(1,2)/_fy});
        }

        template <typename T>
        bool operator()(
            T const* const* unknowns,
            T      *        residuals) const
        {
            T const* rotG        = unknowns[0]; // dim == 2
            T const* a           = unknowns[1]; // dim == 1
            T const* translation = unknowns[2]; // dim == 3
            T const* free        = unknowns[3]; // dim == 3
            T const* q0          = unknowns[4]; // dim == 4
            T const* momentum    = unknowns[5]; // dim == 3
            T const* mass        = unknowns[6]; // dim == 1
            T const* collTime    = unknowns[7]; // dim = 1

            // transform point from origin to point in 3D at time
            T invI[3];
            getInvI(_shape, mass, _sqrSize.data(), invI);
            T timeElapsed = T(_dtHalf) * (T(_observationTime) - collTime[0]);
            T est_q[4];
            integrateQuaternion(q0, momentum, timeElapsed, invI, est_q, _integrationSteps);

            // get offset vector from parabola
            T x[3];
            ParabolaCostFunctor::getPositionAtTime(x, rotG, a, translation, free, T(_observationTime) - collTime[0]);

            T p[3];
            for (size_t i = 0; i != _observations.size(); ++i) {
                // rotate point at origin
                ceres::QuaternionRotatePoint(est_q, unknowns[7+i], p);

                // offset point
                p[0] += x[0];
                p[1] += x[1];
                p[2] += x[2];

                // map to 2D
                if (p[2] == T(0.)) {
                    residuals[i*2  ] = T(1.) - p[2];
                    residuals[i*2+1] = T(1.) - p[2];
                } else {
                    // estimate diff
                    residuals[i*2  ] = p[0] / p[2] - T(_observations[i](0));
                    residuals[i*2+1] = p[1] / p[2] - T(_observations[i](1));
                }
            } //...for each observation

//            residuals[2] = p0[0];
//            residuals[3] = p0[1];
//            residuals[4] = p0[2];

            return true;
        } //...operator()

        template <typename _Matrix3, typename _Scalar>
        static ceres::CostFunction* Create(
            _Matrix3                   const&       intr,
            CeresScalar                const        time,
            CeresScalar                const        dt,
            _Scalar                    const* const size,
            int                        const        collTimeLowerBound,
            int                        const        substepsPerFrame,
            SHAPE                      const        shape,
            std::list<Eigen::Vector2d> const&       observations )
        {
            if (observations.size() > 1)
                std::cout << "[" << __func__ << "] creating functor with " << observations.size() << " observations at time " << time << std::endl;

            auto fn = new ceres::DynamicAutoDiffCostFunction<BatchPointCostFunctor>(new BatchPointCostFunctor(
                intr,time,dt,size,collTimeLowerBound,substepsPerFrame,shape,observations));
            fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_A_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_TRANSLATION_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
            fn->AddParameterBlock(PhysIndexer::POSE_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MASS_STRIDE);
            fn->AddParameterBlock(PhysIndexer::COLL_TIME_STRIDE);
            for (size_t pointId = observations.size(); pointId; --pointId)
                fn->AddParameterBlock(PhysIndexer::POINTS_STRIDE);
            fn->SetNumResiduals(observations.size() * NUM_RESIDUALS);
            return fn;
        }

    protected:
        CeresScalar                  const _fx, _fy;          //!< Camera intrinsics
        CeresScalar                  const _observationTime;  //!< Time of 2D observation (u,v)
        CeresScalar                  const _dtHalf;           //!< Half of timestep size.
        std::array<CeresScalar,3>    const _sqrSize;          //!< Size of the object, used for inertia tensor estimation
        int                          const _integrationSteps; //!< How many integration steps to do for the elapsed time (precomputed).
        SHAPE                        const _shape;            //!< shape flag for inertia tensor
        std::vector<Eigen::Vector2d>       _observations;
}; //...PointCostFunctor

} //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_BATCHPOINTTERM_HPP
