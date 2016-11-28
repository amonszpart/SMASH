//
// Created by bontius on 22/07/16.
//

#include "tracking/common/clouds/impl/cloudFactory.hpp"

template void tracking::CloudFactory::addPoint(
Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 1>, Eigen::RowMajor, 3, true> > const&,
Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 1>, Eigen::RowMajor, 3, true> > const&,
Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 1>, Eigen::RowMajor, 3, true> > const&);
