//
// Created by bontius on 31/01/16.
//

#ifndef TRACKVIDEO_CERESEULERYXY_H
#define TRACKVIDEO_CERESEULERYXY_H

#include "ceres/rotation.h"

namespace ceres
{
  /** \brief Rotates a 3D vector using YXY Euler angles in degrees. */
  template <typename T>
  inline void EulerYXYRotatePoint(const T* const y0, const T* const x, const T* const y1, const T pt[3], T result[3] );

  /** \brief Converts YXY Euler angles in degrees to a 3x3 rotation matrix. */
  template <typename T, int row_stride, int col_stride>
  inline void EulerYXYToRotationMatrix( const T* euler, const ceres::MatrixAdapter<T, row_stride, col_stride>& R);
}

#include "tracking/common/util/impl/ceresEulerYXY.hpp"

#endif //TRACKVIDEO_CERESEULERYXY_H
