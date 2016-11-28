//
// Created by bontius on 22/04/16.
//

#ifndef TRACKVIDEO_COMMON_SPHERICALPARAMETRIZATION_H
#define TRACKVIDEO_COMMON_SPHERICALPARAMETRIZATION_H

#include "tracking/common/eigen.h"
#include <iostream>

namespace tracking {
  template <typename _Scalar>
  inline Eigen::Matrix<_Scalar,3,1> sph2cart(_Scalar const r, _Scalar const theta, _Scalar const phi) {
      if (std::abs(theta) > 2 * M_PI) {
          std::cerr << "[" << __func__ << "] theta is " << theta << ", expected to be in radians..." << std::endl;
      }
      if (std::abs(phi) > 2 * M_PI) {
          std::cerr << "[" << __func__ << "] phi is " << phi << ", expected to be in radians..." << std::endl;
      }
      return { r * std::cos(theta) * std::sin(phi),
               r * std::sin(theta) * std::sin(phi),
               r * std::cos(phi) };
  } //...sph2cart
} //...ns tracking

#endif //TRACKVIDEO_COMMON_SPHERICALPARAMETRIZATION_H
