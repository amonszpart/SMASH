//
// Created by bontius on 20/07/16.
//

#include "tracking/common/clouds/impl/kdCloud.hpp"
#include "tracking/common/clouds/typedefsCloud.h"
#include "ceres/typedefs.h"

template class tracking::KdCloud<tracking::CloudT>;

template std::vector<tracking::LinId>
tracking::KdCloud<tracking::CloudT>::findClosestIds(Eigen::MatrixBase<Eigen::Matrix<Scalar, 3, 1> > const&, int, float) const;

template std::vector<tracking::LinId>
tracking::KdCloud<tracking::CloudT>::findClosestIds(Eigen::MatrixBase<Eigen::Block<
    /*    XprType: */ Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor> const,
    /*  BlockRows: */ 1,
    /*  BlockCols: */ 3,
    /* InnerPanel: */ true> > const&, int, float) const;

template std::vector<tracking::LinId>
tracking::KdCloud<tracking::CloudT>::findClosestIds(Eigen::MatrixBase<Eigen::Matrix<Scalar, 3, 1> > const&,
                                  int,
                                  float,
                                  std::vector<float>&) const;

template std::vector<tracking::LinId>
tracking::KdCloud<tracking::CloudT>::findClosestIds(Eigen::MatrixBase<Eigen::Block<
    /*    XprType: */ Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor> const,
    /*  BlockRows: */ 1,
    /*  BlockCols: */ 3,
    /* InnerPanel: */ true> > const&, int, float, std::vector<float>&) const;

