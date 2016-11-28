//
// Created by bontius on 20/07/16.
//

#ifndef TRACKVIDEO_COMMON_KDCLOUD_HPP
#define TRACKVIDEO_COMMON_KDCLOUD_HPP

#include "tracking/common/clouds/kdCloud.h"

namespace tracking {

template <typename _CloudT>
template <typename _DerivedT>
inline std::vector<LinId>
KdCloud<_CloudT>::findClosestIds(Eigen::MatrixBase<_DerivedT> const& x0,
                                 int const K,
                                 Scalar const inlierThresh,
                                 std::vector<Scalar>& sqrDists) const {
    std::vector<LinId> out;
    Scalar const sqrInlierThresh = inlierThresh * inlierThresh;

    std::vector<size_t> retIndices(K);
    sqrDists.resize(K,std::numeric_limits<Scalar>::max());
    nanoflann::KNNResultSet<Scalar> resultSet(K);

    resultSet.init(&retIndices[0], &sqrDists[0]);
    _tree->index->findNeighbors(resultSet, x0.derived().data(), nanoflann::SearchParams());
    for (size_t i = 0; i != resultSet.size(); ++i) {
        if (inlierThresh > 0. && sqrDists[i] > sqrInlierThresh)
            continue;

        out.push_back(retIndices[i]);
    }

    return out;
} //...KdCloud::findClosestIds()

template <typename _CloudT>
template <typename _DerivedT>
inline std::vector<LinId>
KdCloud<_CloudT>::findClosestIds(Eigen::MatrixBase<_DerivedT> const& x0, int const K, Scalar const inlierThresh) const {
    std::vector<Scalar> sqrDists;
    return findClosestIds(x0,K,inlierThresh,sqrDists);
} //...KdCloud::findClosestIds()

template <typename _CloudT>
KdCloud<_CloudT>::KdCloud(std::shared_ptr<_CloudT const> const& cloud)
    : _cloud(cloud),
      _tree(new KdTreeT{Dim, cloud->getPoints(), maxLeafSize})
{ _tree->index->buildIndex(); }

#if 0
template <typename _CloudT>
KdCloud<_CloudT>::KdCloud(KdCloud const& other)
    : _cloud(other._cloud)
    , _tree(other._tree) {
} //...KdCloud(KdCloud const&)
#endif

} //...ns tracking

#endif //TRACKVIDEO_COMMON_KDCLOUD_HPP
