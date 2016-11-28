//
// Created by bontius on 05/12/15.
//

#ifndef TRACKVIDEO_CERESUTIL_H_H
#define TRACKVIDEO_CERESUTIL_H_H

#include "tracking/common/maths/quaternion.h"
#include "tracking/common/util/ceresEulerYXY.h"
#include "ceres/ceresUtil.h"
#include "ceres/rotation.h"

namespace ceres {

  template <typename T>
  inline void EulerAnglesRotatePoint(const T* const r0, const T* const r1, const T* const r2, const T pt[3], T result[3])  {
      const double kPi = 3.14159265358979323846;
      const T degrees_to_radians(kPi / 180.0);

      const T pitch( r0[0] * degrees_to_radians );
      const T roll ( r1[0] * degrees_to_radians );
      const T yaw  ( r2[0] * degrees_to_radians );

      const T c1 = cos(yaw);
      const T s1 = sin(yaw);
      const T c2 = cos(roll);
      const T s2 = sin(roll);
      const T c3 = cos(pitch);
      const T s3 = sin(pitch);

      result[0] = (c1*c2) * pt[0] + (-s1*c3 + c1*s2*s3) * pt[1] + ( s1*s3 + c1*s2*c3) * pt[2];
      result[1] = (s1*c2) * pt[0] + ( c1*c3 + s1*s2*s3) * pt[1] + (-c1*s3 + s1*s2*c3) * pt[2];
      result[2] = (-s2  ) * pt[0] + ( c2*s3           ) * pt[1] + ( c2*c3           ) * pt[2];

      //        R(0, 0) = c1*c2;  R(0, 1) = -s1*c3 + c1*s2*s3;  R(0, 2) = s1*s3 + c1*s2*c3;
      //        R(1, 0) = s1*c2;  R(1, 1) =  c1*c3 + s1*s2*s3;  R(1, 2) = -c1*s3 + s1*s2*c3;
      //        R(2, 0) = -s2;    R(2, 1) = c2*s3;              R(2, 2) = c2*c3;
  }

  template <typename T>
  inline void EulerAnglesRotatePoint(const T euler[3], const T pt[3], T result[3]) {
      EulerAnglesRotatePoint( euler[0], euler[1], euler[2], pt, result );
  } //...EulerAnglesRotatePoint

#if 0
  template <typename T>
inline void EulerYXYRotatePoint( const T* const r0, const T* const r1, const T* const r2, const T pt[3], T result[3] )
{
    static const double kPi = 3.14159265358979323846;
    static const T degrees_to_radians(kPi / 180.0);

    const T y0(r0[0] * degrees_to_radians);
    const T x (r1[0] * degrees_to_radians);
    const T y1(r2[0] * degrees_to_radians);

    const T cy0 = cos(y0);
    const T sy0 = sin(y0);
    const T cx  = cos(x);
    const T sx  = sin(x);
    const T cy1 = cos(y1);
    const T sy1 = sin(y1);

    //{{cy0*cy1-cx*sy0*sy1,sx*sy1,cy1*sy0+cx*cy0*sy1},
    // {sx*sy0,cx,-cy0*sx},
    // {-cx*cy1*sy0-cy0*sy1,cy1*sx,cx*cy0*cy1-sy0*sy1}}

    result[0] = (cy0*cy1 - cx*sy0*sy1)  * pt[0] + (sx*sy1) * pt[1] + (cy1*sy0 + cx*cy0*sy1) * pt[2];
    result[1] = (sx*sy0)                * pt[0] + (cx)     * pt[1] + (-cy0*sx)              * pt[2];
    result[2] = (-cx*cy1*sy0 - cy0*sy1) * pt[0] + (cy1*sx) * pt[1] + (cx*cy0*cy1 - sy0*sy1) * pt[2];
}
#endif

} //...ns ceres

namespace tracking {
  namespace bundle_physics {

    // The Eigen constructor takes w,x,y,z.
    // Ceres order is also w,x,y,z.
    // Eigen .coeffs() returns x,y,z,w.
    // The Eigen constructor Eigen::Quaternion( Vector4 coeffs ) takes x,y,z,w.

    template<typename _Scalar>
    inline void quaternionToCeres(const pa::Quaternion<_Scalar> &pose, ceres::CeresScalar poseAddress[4]) {
        poseAddress[0] = pose.w();
        poseAddress[1] = pose.x();
        poseAddress[2] = pose.y();
        poseAddress[3] = pose.z();
    } //...quaternionToCeres()

    template<typename _Scalar>
    inline void ceresToQuaternion(const ceres::CeresScalar poseAddress[4], pa::Quaternion<_Scalar> &pose) {
        pose.w() = poseAddress[0];
        pose.x() = poseAddress[1];
        pose.y() = poseAddress[2];
        pose.z() = poseAddress[3];
    } //...ceresToQuaternion()

    template<typename T>
    inline void rotatePoint(const T *const r0, const T r12[2], const T *const p, T *x) {
        ceres::EulerYXYRotatePoint( /* y0: */ r0, /* x: */ r12, /* y1: */ r12 + 1, /* in: */ p, /* out: */ x);
    } //...rotatePoint(0

    template<typename T>
    inline void toRotationMatrix(const T *const r0, const T r12[2], Eigen::Matrix<T, 3, 3, Eigen::ColMajor> &R) {
        T rotation[3];
        rotation[0] = r0 [0];
        rotation[1] = r12[0];
        rotation[2] = r12[1];
        ceres::EulerYXYToRotationMatrix(rotation, ceres::ColumnMajorAdapter3x3(R.data()));
    } //...toRotationMatrix()


    inline ceres::LossFunction* chooseHuberOrTrivial(ceres::CeresScalar const outerScale, ceres::CeresScalar const huberScale) {
        if (huberScale > 0.)
            return new ceres::ScaledLoss(new ceres::HuberLoss(huberScale),outerScale,ceres::TAKE_OWNERSHIP);
        else
            return new ceres::ScaledLoss(nullptr,outerScale,ceres::TAKE_OWNERSHIP);

    }
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_CERESUTIL_H_H
