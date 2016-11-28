//
// Created by bontius on 22/07/16.
//

#ifndef TRACKVIDEO_COMMON_CLOUDFACTORY_HPP
#define TRACKVIDEO_COMMON_CLOUDFACTORY_HPP

#include "tracking/common/clouds/cloudFactory.h"
#include "tracking/common/util/prettyPrint.h"

namespace tracking {

template <typename _DerivedPointT, typename _DerivedNormalT, typename _DerivedColorT>
inline void CloudFactory::addPoint(Eigen::MatrixBase<_DerivedPointT>  const& point,
                                   Eigen::MatrixBase<_DerivedNormalT> const& normal,
                                   Eigen::MatrixBase<_DerivedColorT>  const& color) {
    static_assert(_DerivedPointT::RowsAtCompileTime + _DerivedPointT::ColsAtCompileTime == 4, "3D point expected");
    static_assert(_DerivedNormalT::RowsAtCompileTime + _DerivedNormalT::ColsAtCompileTime == 4, "3D normal expected");
    static_assert(_DerivedColorT::RowsAtCompileTime + _DerivedColorT::ColsAtCompileTime == 4, "3D color expected");
    if ((_points.size() != _normals.size()) || (_normals.size() != _colors.size()))
        throw new CloudFactory_MixedCallsException();

//    _points.push_back(Vector3{point.coeff(0),point.coeff(1),point.coeff(2)});
    _points.push_back(point);
    _normals.push_back(normal);
    _colors.push_back(color);
}
} //...ns tracking

#endif //TRACKVIDEO_COMMON_CLOUDFACTORY_HPP
