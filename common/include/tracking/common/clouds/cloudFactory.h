//
// Created by bontius on 15/03/16.
//

#ifndef TRACKVIDEO_COMMON_CLOUDFACTORY_H
#define TRACKVIDEO_COMMON_CLOUDFACTORY_H

#include "tracking/common/eigen.h"
#include "tracking/common/clouds/typedefsCloud.h"
#include "tracking/common/util/exception.h"
#include "soup/pointCloudEigen.h"
#include <vector>

namespace tracking {
DEFINE_EXCEPTION(CloudFactory_MixedCalls)
DEFINE_EXCEPTION(CloudFactory_SizesDontMatch)

class CloudFactory {
public:
    using Vector3 = typename CloudT::Vector3;
    using Scalar = typename CloudT::Scalar;

    inline void addPoint(Vector3 point) {
        if (_normals.size() || _colors.size())
            throw new CloudFactory_MixedCallsException();

        _points.emplace_back(std::move(point));
    } //...addPoint()

    inline void addPointWithNormal(Vector3 point, Vector3 normal) {
        if (_colors.size() || (_points.size() != _normals.size()))
            throw new CloudFactory_MixedCallsException();

        _points.emplace_back(std::move(point));
        _normals.emplace_back(std::move(normal));
    } //...addPoint()

    inline void addPointWithColor(Vector3 point, Vector3 color) {
        if (_normals.size() || (_points.size() != _colors.size()))
            throw new CloudFactory_MixedCallsException();

        _points.emplace_back(std::move(point));
        _colors.emplace_back(std::move(color));
    } //...addPoint()

    inline void addPoint(Vector3 point, Vector3 normal, Vector3 color) {
        if ((_points.size() != _normals.size()) || (_normals.size() != _colors.size()))
            throw new CloudFactory_MixedCallsException();

        _points.emplace_back(std::move(point));
        _normals.emplace_back(std::move(normal));
        _colors.emplace_back(std::move(color));
    } //...addPoint()
#if 1
    template <typename _DerivedPointT, typename _DerivedNormalT, typename _DerivedColorT>
    void addPoint(Eigen::MatrixBase<_DerivedPointT> const& point,
                  Eigen::MatrixBase<_DerivedNormalT> const& normal,
                  Eigen::MatrixBase<_DerivedColorT> const& color);
#endif

    /** \brief Returns many points are currently held. */
    inline size_t size() const {return _points.size();}

    /** \brief Creates a cloud from the currently held information. */
    inline CloudPtrT create() const {
        if ( (_normals.size() && (_points.size() != _normals.size())) ||
             (_colors .size() && (_points.size() != _colors .size())) )
            throw new CloudFactory_SizesDontMatchException();

        CloudPtrT cloud {new CloudT(_points.size())};
        for (size_t row = 0; row != _points.size(); ++row) {
            cloud->getPoint(row) = _points[row];
            if (row < _normals.size())
                cloud->getNormal(row) = _normals[row];
            if (row < _colors.size())
                cloud->getColor(row) = _colors[row];
        }
        if (!_normals.size())
            cloud->clearNormals();
        if (!_colors.size())
            cloud->clearColors();
        return cloud;
    } //...create()

protected:
    std::vector<Vector3> _points;
    std::vector<Vector3> _normals;
    std::vector<Vector3> _colors;
public:
//    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
}; //...class CloudFactory
} //...ns tracking

#endif //TRACKVIDEO_COMMON_CLOUDFACTORY_H
