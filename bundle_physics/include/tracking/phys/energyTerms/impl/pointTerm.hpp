//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_POINTTERM_HPP
#define TRACKVIDEO_POINTTERM_HPP

#include "tracking/phys/energyTerms/impl/poseTerm.hpp"
#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/inertiaEstimation.hpp"
#include "tracking/phys/physIndexer.h"
#include "ceres/rotation.h"

namespace tracking {
  namespace bundle_physics {

    /** \addtogroup Physics
     * @{ */

    /** \addtogroup PhysicsFormulation
     *  @{
     */

    struct PointCostFunctor {
        public:
            enum { NUM_RESIDUALS = 2 };
            using CeresScalar = ceres::CeresScalar;
            typedef ceres::AutoDiffCostFunction< PointCostFunctor,
                NUM_RESIDUALS,
                PhysIndexer::POINTS_STRIDE,
                PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE,
                PhysIndexer::PARABOLA_SHARED_A_STRIDE,
                PhysIndexer::PARABOLA_TRANSLATION_STRIDE,
                PhysIndexer::PARABOLA_FREE_STRIDE,
                PhysIndexer::POSE_STRIDE,
                PhysIndexer::MOMENTUM_STRIDE,
                PhysIndexer::MASS_STRIDE,
                PhysIndexer::COLL_TIME_STRIDE
            > AutoDiffType;

            template <typename _Scalar, typename _Matrix3>
            PointCostFunctor(
                _Matrix3    const&       intr,
                CeresScalar const        u,
                CeresScalar const        v,
                CeresScalar const        time,
                CeresScalar const        dt,
                _Scalar     const* const size,
                int         const        collTimeLowerBound,
                float         const        substepsPerFrame,
                SHAPE       const        shape
            );

            template <typename T>
            bool operator()(
                const T p0[3],
                const T rotG[2],
                const T a[1],
                const T translation[3],
                const T free[3],
                const T q0[4],
                const T momentum[3],
                const T mass[1],
                const T collTime[1],
                T* residuals ) const;


            template <typename _Scalar, typename _Matrix3>
            static ceres::CostFunction* Create(
                _Matrix3    const&       intr,
                CeresScalar const        u,
                CeresScalar const        v,
                CeresScalar const        time,
                CeresScalar const        dt,
                _Scalar     const* const size,
                int         const        collTimeLowerBound,
                float       const        substepsPerFrame,
                SHAPE       const        shape);

            void setIntegrationSteps(int integrationSteps) {_integrationSteps = integrationSteps;}
            int getIntegrationSteps() const { return _integrationSteps; }

        protected:
            const CeresScalar                _fx, _fy;          //!< Camera intrinsics
            const CeresScalar                _u, _v;            //!< Observed 2D location of point
            const CeresScalar                _observationTime;  //!< Time of 2D observation (u,v)
            const CeresScalar                _dtHalf;           //!< Half of timestep size.
            const std::array<CeresScalar, 3> _sqrSize;          //!< Size of the object, used for inertia tensor estimation
            /* */ int                        _integrationSteps; //!< How many integration steps to do for the elapsed time (precomputed).
            const SHAPE                      _shape;            //!< shape flag for inertia tensor
    }; //...struct PointCostFunctor

    /** @} */
    /** @} (PhysicsFormulation) */

  } //...ns bundle_phsyics
} //...ns tracking

namespace tracking {
  namespace bundle_physics {
    template <typename _Scalar, typename _Matrix3>
    PointCostFunctor::PointCostFunctor(
        _Matrix3    const&       intr,
        CeresScalar const        u,
        CeresScalar const        v,
        CeresScalar const        time,
        CeresScalar const        dt,
        _Scalar     const* const size,
        int         const        collTimeLowerBound,
        float       const        substepsPerFrame,
        SHAPE       const        shape
    )
        : _fx( intr(0,0) ),
        _fy( intr(1,1) ),
        _u((u - intr(0,2))/_fx),
        _v((v - intr(1,2))/_fy),
        _observationTime(time ),
        _dtHalf( dt / 2. ),
        _sqrSize{size[0] * size[0], size[1] * size[1], size[2] * size[2]},
        _integrationSteps(std::round(substepsPerFrame * std::max(1.,std::abs(collTimeLowerBound - _observationTime)))),
        _shape( shape )
    {}

    /**
     * \param[in]  p0          3D point location in object space.
     * \param[in]  rotG        The two later rotation angles of corresponding to the gravity vector ({\f$ \beta_{x},\beta_{y1} \f$} from {\f$ \beta_{y0}, \beta_{x}, \beta_{y1} \f$}).
     * \param[in]  a           Squared parameter of parabola (gravity scale), aka \f$ b_1 \f$.
     * \param[in]  translation 3D translation of parabola (position of object at collision time), aka \f$ b_4 \f$.
     * \param[in]  free        Free parameters of parabola. Angle "y0" (\f$\beta_{y0}\f$) in degrees, linear term "b" (\f$b_2\f$), and x scaling "s" (\f$b_3\f$).
     * \param[in]  q0          Collision pose (quaternion), \f$ \mathbf{q}^c \f$.
     * \param[in]  momentum    Momentum of object, aka \f$ \mathbf{k}^{pre/post,~a/b} \f$.
     * \param[in]  mass        Mass of object, \f$ m^a \f$ or \f$ m^b \f$.
     * \param[in]  collTime    Time of collision, \f$ t^c \f$.
     */
    template <typename T>
    bool PointCostFunctor::operator()(
        const T p0[3],
        const T rotG[2],
        const T a[1],
        const T translation[3],
        const T free[3],
        const T q0[4],
        const T momentum[3],
        const T mass[1],
        const T collTime[1],
        T* residuals ) const
    {
        T p[3];
#if 1
        // transform point from origin to point in 3D at time
        T invI[3];
        getInvI(_shape, mass, _sqrSize.data(), invI);

        T timeElapsed = T(_dtHalf) * (T(_observationTime) - collTime[0]);
        T est_q[4];
        integrateQuaternion(q0, momentum, timeElapsed, invI, est_q, std::max(1,_integrationSteps));

        // rotate point at origin
        ceres::QuaternionRotatePoint(est_q, p0, p);
#else
        p[0] = p0[0];
        p[1] = p0[1];
        p[2] = p0[2];
#endif

        // get offset vector from parabola
        T x[3];
        ParabolaCostFunctor::getPositionAtTime(x, rotG, a, translation, free, T(_observationTime) - collTime[0]);
        // offset point
        p[0] += x[0];
        p[1] += x[1];
        p[2] += x[2];

        // map to 2D
        if ( p[2] == T(0.) )
            return false;

        // estimate diff
        residuals[0] = p[0] / p[2] - T(_u);
        residuals[1] = p[1] / p[2] - T(_v);

        //residuals[2] = 0.001 * (x[0] / x[2] - T(_u));
        //residuals[3] = 0.001 * (x[1] / x[2] - T(_v));

        return true;
    } //...operator()

    template <typename _Scalar, typename _Matrix3>
    ceres::CostFunction* PointCostFunctor::Create(
        _Matrix3    const&       intr,
        CeresScalar const        u,
        CeresScalar const        v,
        CeresScalar const        time,
        CeresScalar const        dt,
        _Scalar     const* const size,
        int         const        collTimeLowerBound,
        float       const        substepsPerFrame,
        SHAPE       const        shape)
    {
        return new AutoDiffType( new PointCostFunctor(intr,u,v,time,dt,size,collTimeLowerBound,substepsPerFrame,shape) );
    }

  } //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_POINTTERM_HPP
