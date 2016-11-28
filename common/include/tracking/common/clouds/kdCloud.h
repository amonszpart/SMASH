//
// Created by bontius on 25/05/16.
//

#ifndef TRACKVIDEO_FUNC_KDCLOUD_H
#define TRACKVIDEO_FUNC_KDCLOUD_H

#include "tracking/common/eigen.h"
#include "tracking/common/clouds/kdCloudFwDecl.h"
#include "tracking/common/clouds/typedefsCloud.h"
#include "soup/pointCloudEigen.h"
#include "nanoflann/nanoflann.hpp"
#include <list>
#include <memory>

namespace tracking {

/** \brief Wrapper class for a pointcloud and its kdtree. */
template <typename _CloudT>
class KdCloud {
public:
    static constexpr int maxLeafSize = 10;
    enum { Dim = _CloudT::CloudT::Dim};
    typedef _CloudT CloudT;

    /** \brief Type that hashes a \ref PointCloud. That means only positions, Nx3. */
    typedef nanoflann::KDTreeEigenMatrixAdaptor<typename _CloudT::CloudT> KdTreeT;
    using Scalar = typename _CloudT::Scalar;

    /** \brief Constructor from the PointCloud to hash. PointCloud holds only points.*/
    explicit KdCloud(std::shared_ptr<_CloudT const> const& cloud);

    /** \brief Move constructor (default). */
    explicit KdCloud(KdCloud &&) = default;

    /** \brief Copy constructor. */
    explicit KdCloud(KdCloud const&) = default; // used in CloudProximityTerm

    /** \brief Move assignment (default). */
    KdCloud& operator=(KdCloud&&) = default;

    /** \brief Copy assignment (default). */
    KdCloud& operator=(KdCloud const&) = default;

    /** \brief Cloud getter. */
    std::shared_ptr<_CloudT const> const& getCloud() const { return _cloud; }

    /** \brief Nanotree getter. */
    KdTreeT const& getTree() const { return *_tree; }

    template<typename _DerivedT>
    std::vector<LinId>
    findClosestIds(Eigen::MatrixBase<_DerivedT> const& x0, int const K, Scalar const inlierThresh = Scalar{-1.}) const;

    template<typename _DerivedT>
    std::vector<LinId>
    findClosestIds(Eigen::MatrixBase<_DerivedT> const& x0, int const K, Scalar const inlierThresh,
                   std::vector<Scalar>& sqrDists) const;

protected:
    std::shared_ptr<_CloudT const> _cloud;
    std::shared_ptr<KdTreeT const> _tree;
    //std::unique_ptr<KdTreeT> _tree;
};//...class KdCloud

} //...ns tracking

#endif //TRACKVIDEO_KDCLOUD_H
