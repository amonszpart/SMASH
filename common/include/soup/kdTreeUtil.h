#if SOUP_WITH_FLANN
/*
 * Created by: Aron Monszpart (aron.monszpart@gmail.com)
 * On: 8/31/2015
 *
 * Property of Adobe Inc.
*/

#ifndef SOUP_KDTREEUTIL_H
#define SOUP_KDTREEUTIL_H

#include "tracking/common/util/exception.h"
#include "flann/flann.hpp"
#include <memory>
#include <cstdlib>
#include <cmath>
#include <vector>

namespace Soup {
  typedef float ScalarType;
  typedef ScalarType                                      DistType;
  typedef flann::Index<flann::L2<DistType> >             KDTreeType;
  typedef flann::Matrix<ScalarType>                       KDTreeDataSetType;
  typedef std::vector< std::vector<int> >                 FlannIndicesT;
  typedef std::vector< std::vector<DistType> >            FlannDistsT;
  typedef std::shared_ptr<KDTreeType>                     KDTreeTypePtr;

  DEFINE_EXCEPTION( KDTreeNullPtr )

  template <typename _CloudT>
  KDTreeTypePtr buildKdTree( const _CloudT         & cloud );
  //KDTreeTypePtr buildKdTree( const VerticesListType& cloud );

  /** \brief Look for \p K neighbours of a point with id \p pid in source \p cloud in target cloud \p kdtree. */
  template <typename _CloudT>
  int knnSearch   ( const KDTreeType               &kdtree
                    , const _CloudT                  &cloud
                    , const size_t                    pid
                    ,       FlannIndicesT            &indicesList
                    ,       FlannDistsT              &distsList
                    , const size_t                    K
                    , const flann::SearchParams      &searchParams
  );

  template <typename _CloudT>
  int knnSearchCloud( const KDTreeType               &kdtree
                      , const _CloudT                  &cloud
                      ,       FlannIndicesT            &indicesList
                      ,       FlannDistsT              &distsList
                      , const size_t                    K
                      , const typename _CloudT::Scalar  radius
                      , const flann::SearchParams      &searchParams
  );


  /** \brief Look for all neighbours of a point with id \p pid in source \p cloud in target cloud \p kdtree up to \p radius distance. */
  template <typename _CloudT>
  int radiusSearch( const KDTreeType               & kdtree
                    , const _CloudT                  & cloud
                    , const size_t                     pid
                    ,       FlannIndicesT            & indicesList
                    ,       FlannDistsT              & distsList
                    , const typename _CloudT::Scalar   radius
                    , const flann::SearchParams      & searchParams
  );

  template <typename _PointT>
  inline int radiusSearch( const KDTreeType               & kdtree
                           , const _PointT                  & point
                           ,       FlannIndicesT            & indicesList
                           ,       FlannDistsT              & distsList
                           , const typename _PointT::Scalar   radius
                           , const flann::SearchParams      & searchParams = flann::SearchParams()
  );

  /** \brief Look for all neighbours of all points of source \p cloud in target \p kdtree up to \p radius distance. */
  template <typename _CloudT>
  int radiusSearchCloud( const KDTreeType               & kdtree
                         , const _CloudT                  & cloud
                         ,       FlannIndicesT            & indicesList
                         ,       FlannDistsT              & distsList
                         , const typename _CloudT::Scalar   radius
                         , const flann::SearchParams      & searchParams
  );


  template <typename _PointT>
  size_t getNNtopK( const KDTreeType                           * kdtree
                    , const _PointT                              & pnt
                    ,       FlannIndicesT                        & indices
                    ,       FlannDistsT                          & distSqrs
                    , const int                                    K
  );
#if 0
  size_t getNNtopK( const KDTreeType                           * kdtree
                    , const VecType                              & pnt
                    ,       std::vector<int>                     & indices
                    ,       std::vector<KDTreeType::DistanceType>& distSqrs
                    , const int                                    K
                    , const KDTreeType::DistanceType               maxDist
  );

  size_t getNNtopK( const KDTreeType                        *kdtree
                    , const SHOTDescType                      &central_point
                    ,       std::vector<int>                  &ourIndices
                    ,       std::vector<SHOTDescType::Scalar> &outDistsSqr
                    , const int                                K
                    , const SHOTDescType::Scalar               maxRadiusSqr
                    , const size_t                             shotDescLength
                    );
#endif

} //...Soup

#endif // SOUP_KDTREEUTIL_H

#endif // SOUP_WITH_FLANN