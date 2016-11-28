//
// Created by bontius on 31/01/16.
//

#ifndef TRACKVIDEO_CERESEULERYXY_HPP
#define TRACKVIDEO_CERESEULERYXY_HPP

#include "tracking/common/util/ceresEulerYXY.h"

namespace ceres {
  /**
   * \param[in]  y0     First Euler angle in degrees in the YXY convention representing rotation around the y axis.
   * \param[in]  x      Second Euler angle in degrees in the YXY convention representing rotation around the x axis.
   * \param[in]  y1     Third Euler angle in degrees in the YXY convention representing rotation around the y axis.
   * \param[in]  pt     Input vector to rotate.
   * \param[out] result Rotated vector.
   */
  template <typename T>
  inline void EulerYXYRotatePoint(const T* const y0_deg, const T* const x_deg, const T* const y1_deg, const T pt[3], T result[3] )
  {
      static const double kPi = 3.14159265358979323846;
      static const T degrees_to_radians(kPi / 180.0);

      const T y0(y0_deg[0] * degrees_to_radians);
      const T x ( x_deg[0] * degrees_to_radians);
      const T y1(y1_deg[0] * degrees_to_radians);

      const T cy0 = cos(y0);
      const T sy0 = sin(y0);
      const T cx  = cos(x);
      const T sx  = sin(x);
      const T cy1 = cos(y1);
      const T sy1 = sin(y1);

      result[0] = (cy0*cy1 - cx*sy0*sy1)  * pt[0] + (sx*sy1) * pt[1] + (cy1*sy0 + cx*cy0*sy1) * pt[2];
      result[1] = (sx*sy0)                * pt[0] + (cx)     * pt[1] + (-cy0*sx)              * pt[2];
      result[2] = (-cx*cy1*sy0 - cy0*sy1) * pt[0] + (cy1*sx) * pt[1] + (cx*cy0*cy1 - sy0*sy1) * pt[2];
  } //...EulerYXYRotatePoint

  template <typename T, int row_stride, int col_stride>
  inline void EulerYXYToRotationMatrix( const T* euler, const ceres::MatrixAdapter<T, row_stride, col_stride>& R)
  {
      static const double kPi = 3.14159265358979323846;
      static const T degrees_to_radians(kPi / 180.0);

      const T y0(euler[0] * degrees_to_radians);
      const T x (euler[1] * degrees_to_radians);
      const T y1(euler[2] * degrees_to_radians);

      const T cy0 = cos(y0);
      const T sy0 = sin(y0);
      const T cx  = cos(x);
      const T sx  = sin(x);
      const T cy1 = cos(y1);
      const T sy1 = sin(y1);

      R(0, 0) = cy0*cy1 - cx*sy0*sy1;
      R(0, 1) = sx*sy1;
      R(0, 2) = cy1*sy0 + cx*cy0*sy1;

      R(1, 0) = sx*sy0;
      R(1, 1) = cx;
      R(1, 2) = -cy0*sx;

      R(2, 0) = -cx*cy1*sy0-cy0*sy1;
      R(2, 1) = cy1*sx;
      R(2, 2) = cx*cy0*cy1-sy0*sy1;
  } //...EulerYXYToRotationMatrix

} //...ns ceres

#endif //TRACKVIDEO_CERESEULERYXY_HPP
