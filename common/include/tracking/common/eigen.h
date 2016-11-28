//
// Created by bontius on 01/03/16.
//
// "Predefined header" - needs to be included before anything else

#ifndef TRACKVIDEO_EIGEN_H
#define TRACKVIDEO_EIGEN_H

#include "Eigen/Geometry"
#include "Eigen/StdVector"

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2d)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3f)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector3d)

#endif //...TRACKVIDEO_EIGEN_H
