//
// Created by bontius on 05/12/15.
//

#ifndef TRACKVIDEO_PHYSFUNCTORS_H
#define TRACKVIDEO_PHYSFUNCTORS_H

#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"
#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/ceresUtil.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/inertiaEstimation.hpp"
#include "tracking/vis/visualizer.h"
#include "tracking/common/util/energyPlotter.h"
#include "tracking/common/typedefs.h"
#include "ceres/rotation.h"
#include "ceres/local_parameterization.h"


#define QUAT_DIFF 0

namespace tracking {
  namespace bundle_physics {

    template<typename T, typename T2>
    inline T quaternionDiff(const T *q0, const T2 *const q1) {
//        return ceres::acos( T(2.) * (q0[0] * T(q1[0]) +
//                                     q0[1] * T(q1[1]) +
//                                     q0[2] * T(q1[2]) +
//                                     q0[3] * T(q1[3])) - T(1.) );
        //std::cout << ceres::printJetVector4( q0 ) << " - " << ceres::printJetVector4( q1 ) << " = ";
        //T ret = (q0[0] * T(q1[0]) +
//                                q0[1] * T(q1[1]) +
//                                q0[2] * T(q1[2]) +
//                                q0[3] * T(q1[3])) - T(1.);
        //std::cout << ceres::printJet( ret ) << std::endl;
        //return ret;
#if 0
        T dot = T(2.) * (q0[0] * T(q1[0]) +
     q0[1] * T(q1[1]) +
     q0[2] * T(q1[2]) +
     q0[3] * T(q1[3])) - T(1.);
    if ( ceres::abs(dot) < T(1.) )
        return ceres::acos( dot );
    else
    {
        std::cout << ceres::printJet(dot) << ", acos(" << T(2.) * dot - T(1.) << ")" << std::endl;
        std::cout << ceres::printJetVector4( q0 ) << " . " << ceres::printJetVector4( q1 ) << std::endl;
        return T(10.);
    }
#else
        T dot = q0[0] * T(q1[0]) +
                q0[1] * T(q1[1]) +
                q0[2] * T(q1[2]) +
                q0[3] * T(q1[3]);
//        return ceres::abs(dot) - T(1.);
        return T(1.) - dot * dot;
#endif
    }

    /** \brief Estimates current omega from pose and momentum.
     *  \param[in] q0       Pose quaternion in ceres order: w,x,y,z.
     *  \param[in] momentum Angular momentum: \f$ \mathbf{L} = x,y,z. \f$
     *  \param[in] invI     Inverse inertia tensor diagonal: \f$ 1./I_{0,0}, 1./I_{1,1}, 1./I_{2,2}. \f$
     *  \param[out] omega   Instantenous omega as quaternion: \f$ \omega = 0, x,y,z \f$
     */
    template<typename T, typename T2>
    inline void estimateOmegaFromR(const T R[9], const T *momentum, const T2 *invI, T *const omega) {
        // Current angular velocity = R * I^-1 * R^T * momentum
        T momentumDotRCol0 = momentum[0] * R[0] + momentum[1] * R[3] + momentum[2] * R[6];
        T momentumDotRCol1 = momentum[0] * R[1] + momentum[1] * R[4] + momentum[2] * R[7];
        T momentumDotRCol2 = momentum[0] * R[2] + momentum[1] * R[5] + momentum[2] * R[8];

        omega[0] = T(0.);
        omega[1] = R[0] * T(invI[0]) * momentumDotRCol0 + R[1] * T(invI[1]) * momentumDotRCol1 + R[2] * T(invI[2]) * momentumDotRCol2;
        omega[2] = R[3] * T(invI[0]) * momentumDotRCol0 + R[4] * T(invI[1]) * momentumDotRCol1 + R[5] * T(invI[2]) * momentumDotRCol2;
        omega[3] = R[6] * T(invI[0]) * momentumDotRCol0 + R[7] * T(invI[1]) * momentumDotRCol1 + R[8] * T(invI[2]) * momentumDotRCol2;
    }

    /** \brief Estimates current omega from pose and momentum.
     *  \param[in] q0       Pose quaternion in ceres order: w,x,y,z.
     *  \param[in] momentum Angular momentum: \f$ \mathbf{L} = x,y,z. \f$
     *  \param[in] invI     Inverse inertia tensor diagonal: \f$ 1./I_{0,0}, 1./I_{1,1}, 1./I_{2,2}. \f$
     *  \param[out] omega   Instantenous omega as quaternion: \f$ \omega = 0, x,y,z \f$
     */
    template<typename T, typename T2>
    inline void estimateOmega(const T q0[4], const T *momentum, const T2 *invI, T *const omega) {
        T R[9];
        ceres::QuaternionToRotation(q0, ceres::RowMajorAdapter3x3(R));

        estimateOmegaFromR(R, momentum, invI, omega);
    }

    template<typename T, typename T2>
    inline void applyOmega(const T q0[4], const T omega[4], const T2 dtHalf, T est_q1[4]) {
        // q = (q + ( _Quat(_Scalar(0.5) * omega * q) * dt )).normalized();
        // Next rotation = old + dt / 2. * quatMult( omega, prevRotation )
        Eigen::Map<Eigen::Matrix<T, 4, 1> const> map(q0);
        Eigen::Matrix<T,4,1> q0Normalized = map.normalized();
//        if (ceres::abs(1. - map.norm()) > kSmallDiff) {
//            std::cerr << "[" << __func__ << "] " << "q0.norm: " << map.norm() << std::endl;
//        }

        ceres::QuaternionProduct(omega, q0Normalized.data(), est_q1);

        est_q1[0] = q0Normalized(0) + est_q1[0] * T(dtHalf);
        est_q1[1] = q0Normalized(1) + est_q1[1] * T(dtHalf);
        est_q1[2] = q0Normalized(2) + est_q1[2] * T(dtHalf);
        est_q1[3] = q0Normalized(3) + est_q1[3] * T(dtHalf);

        T len = ceres::length(est_q1[0], est_q1[1], est_q1[2], est_q1[3]);
        if (len != T(0.)) {
            len = T(1.)/len;
            est_q1[0] *= len;
            est_q1[1] *= len;
            est_q1[2] *= len;
            est_q1[3] *= len;
        }
    }

    template<typename T, typename T2>
    inline void integrateQuaternion(const T *q0, const T *momentum, const T2 dtHalf, const T *invI, T *est_q1, bool) {
        T omega[4];
        estimateOmega(q0, momentum, invI, omega);
        applyOmega(q0, omega, dtHalf, est_q1);
    } //...integrateQuaternion()

    template<typename T, typename T2>
    inline void integrateQuaternion(const T *q0, const T *momentum, const T2 dtHalf, const T *invI, T *est_q1, int substeps, bool debug = false) {
        if (substeps == 1)
            integrateQuaternion(q0, momentum, dtHalf, invI, est_q1, debug);
        else {
            T a[4], b[4];
            T (*ping)[4] = &a;
            T (*pong)[4] = &b;
            const T2 subDtHalf(dtHalf / T2(substeps));
            integrateQuaternion(q0, momentum, subDtHalf, invI, a, debug);
            for (int i = 1; i < substeps - 1; ++i) {
                integrateQuaternion(*ping, momentum, subDtHalf, invI, *pong, debug);
                std::swap(ping, pong);
            }
            integrateQuaternion(*ping, momentum, subDtHalf, invI, est_q1, debug);
        }
    } //...integrateQuaternion()

    struct PoseIntegralCostFunctor {
            typedef ceres::CeresScalar CeresScalar;

#if QUAT_DIFF
            enum { NUM_RESIDUALS = 1 };
#else
            enum { NUM_RESIDUALS = 4 };
#endif

            /**
             *  \param[in] sqrtWeight   Sqrt of term weight.
             *  \param[in] dt           Timestep size.
             *  \param[in] queryFrameId Time between frameId and the equality constraint.
             */
            template<typename _Scalar, typename _Vector4>
            PoseIntegralCostFunctor(
                CeresScalar const        sqrtWeight,
                CeresScalar const        dt,
                _Scalar     const* const size,
                _Vector4    const&       observedPose,
                CeresScalar const        queryFrameId,
                int         const        collTimeLowerBound,
                float       const        substepsPerFrame,
                SHAPE       const        shape )
                : _sqrtWeight(sqrtWeight),
                  _dtHalf(dt / 2.),
                  _sqrSize{size[0] * size[0], size[1] * size[1], size[2] * size[2]},
                  _queryFrameId(queryFrameId),
                  _observedWXYZ(observedPose(3), observedPose(0), observedPose(1), observedPose(2)),
//                _integrationSteps(substepsPerFrame * std::max(1.,std::abs(collTimeLowerBound - _queryFrameId))),
                  _integrationSteps(std::round(substepsPerFrame * std::max(1.,std::abs(collTimeLowerBound - queryFrameId)))),
                  _shape(shape)
            {
                if (std::abs(1. - observedPose.norm()) > kSmallDiff) {
                    std::cerr << "[" << __func__ << "] " << "not normalized keyframe pose..." << observedPose.transpose() << ", norm: " << observedPose.norm() << std::endl;
                }
                _observedWXYZ.normalize();
            }

            template<typename T>
            bool operator()(
                const T  q0[4],         // w,x,y,z
                const T  momentum[3],   // x,y,z
                const T  mass[1],
                const T  collTime[1],
                T* residuals) const
            {
                T invI[3];
                getInvI(_shape, mass, _sqrSize.data(), invI);

                T timeElapsed = T(_dtHalf) * (T(_queryFrameId) - collTime[0]);
                T est_q[4];
                integrateQuaternion(q0, momentum, timeElapsed, invI, est_q, static_cast<int>(std::round(std::max(1.f,_integrationSteps))));
                Eigen::Map<const Eigen::Matrix<T,4,1>> estPose(est_q);
                if (ceres::abs(T(1.) - estPose.norm()) > kSmallDiff) {
                    std::cerr << "[" << __func__ << "] estPose norm:" << estPose.norm() << std::endl;
                }

#if QUAT_DIFF
                residuals[0] = T(_sqrtWeight) * quaternionDiff( est_q, _observedWXYZ.data() );
#elif 0
                T r0[4], r1[4];

                r0[0] = (est_q[0] - _observedWXYZ[0]);
                r0[1] = (est_q[1] - _observedWXYZ[1]);
                r0[2] = (est_q[2] - _observedWXYZ[2]);
                r0[3] = (est_q[3] - _observedWXYZ[3]);

                r1[0] = (est_q[0] + _observedWXYZ[0]);
                r1[1] = (est_q[1] + _observedWXYZ[1]);
                r1[2] = (est_q[2] + _observedWXYZ[2]);
                r1[3] = (est_q[3] + _observedWXYZ[3]);
                if (ceres::length4(r0) < ceres::length4(r1)) {
                    residuals[0] = T(_sqrtWeight) * r0[0];
                    residuals[1] = T(_sqrtWeight) * r0[1];
                    residuals[2] = T(_sqrtWeight) * r0[2];
                    residuals[3] = T(_sqrtWeight) * r0[3];
//                    std::cout << "[" << __func__ << "] " << "using r0" << std::endl;
                } else {
                    residuals[0] = T(_sqrtWeight) * r1[0];
                    residuals[1] = T(_sqrtWeight) * r1[1];
                    residuals[2] = T(_sqrtWeight) * r1[2];
                    residuals[3] = T(_sqrtWeight) * r1[3];
//                    std::cout << "[" << __func__ << "] " << "using r1" << std::endl;
                }
#else
                residuals[0] = T(_sqrtWeight) * (est_q[0] - _observedWXYZ[0]);
                residuals[1] = T(_sqrtWeight) * (est_q[1] - _observedWXYZ[1]);
                residuals[2] = T(_sqrtWeight) * (est_q[2] - _observedWXYZ[2]);
                residuals[3] = T(_sqrtWeight) * (est_q[3] - _observedWXYZ[3]);
#endif

                return true;
            } //...operator()

            // Factory to hide the construction of the CostFunction object from the client code.
            template<typename _Scalar, typename _Vector4>
            static ceres::CostFunction *Create(const double sqrtWeight, double dt, const _Scalar *const size, const _Vector4 &observedPose, const ceres::CeresScalar queryFrameId
                                               , const int collTimeLowerBound, const float substepsPerFrame, const SHAPE shape )
            {
                return (new ceres::AutoDiffCostFunction<PoseIntegralCostFunctor, NUM_RESIDUALS,
                    PhysIndexer::POSE_STRIDE, PhysIndexer::MOMENTUM_STRIDE, PhysIndexer::MASS_STRIDE, PhysIndexer::COLL_TIME_STRIDE>(
                    new PoseIntegralCostFunctor(sqrtWeight, dt, size, observedPose, queryFrameId, collTimeLowerBound, substepsPerFrame, shape)));
            }

        protected:
            const CeresScalar                _sqrtWeight;       //!< Sqrt of term weight.
            const CeresScalar                _dtHalf;           //!< Half of timestep size.
            const std::array<CeresScalar, 3> _sqrSize;          //!< Size of the object, used for inertia tensor estimation
            const CeresScalar                _queryFrameId;     //!< At what time the pose equality has to hold.
//            const std::array<CeresScalar, 4> _observedWXYZ;     //!< Pose observation we want to be as close as possible to.
            Eigen::Matrix<CeresScalar,4,1> _observedWXYZ;     //!< Pose observation we want to be as close as possible to.
            const float                      _integrationSteps; //!< How many integration steps to do for the elapsed time (precomputed).
            // Careful! If a long time passed between the collision time and the observation (_queryFrameId),
            // this raise the unknowns to very high exponents.
            const SHAPE                      _shape;            //!< shape flag for inertia tensor
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    }; //...PoseIntegralCostFunctor

  } //...ns bundle_physics
} //...ns tracking


#endif //TRACKVIDEO_PHYSFUNCTORS_H