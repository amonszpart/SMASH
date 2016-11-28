//
// Created by bontius on 28/03/16.
//

#ifndef TRACKVIDEO_PHYS_TYPEDEFSGEOMETRY_H
#define TRACKVIDEO_PHYS_TYPEDEFSGEOMETRY_H

#include "tracking/common/eigen.h"
#include "tracking/common/maths/quaternion.h"

namespace tracking {
  namespace bundle_physics {
    typedef float                                       Scalar;
    typedef Eigen::Matrix<Scalar,3,1>                   Vector3;        //!< 3D position in space.
    typedef pa::Quaternion<Scalar>                      QuaternionT;
    typedef Eigen::Translation<Scalar,3>                TranslationT;
    typedef Eigen::Transform  <Scalar,3, Eigen::Affine> TransformationT;
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_TYPEDEFSGEOMETRY_H
