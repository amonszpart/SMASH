/*
 * Created by: Aron Monszpart (aron.monszpart@gmail.com)
 * On: 8/31/2015
 *
 * Property of Adobe Inc.
*/

#ifndef KDTREEUTIL_HPP
#define KDTREEUTIL_HPP

#include "soup/structures/kdTreeUtil.h"
#include <vector>

namespace Soup
{
    template <typename _CloudT>
    inline std::shared_ptr<KDTreeType> buildKdTree( const _CloudT& cloud )
    {
        std::shared_ptr<KDTreeType> kdtree( nullptr );
        // convert to flann compatible data structure and build kd tree
        typedef KDTreeDataSetType::type MxScalarT;
        static_assert( std::is_same<MxScalarT, typename _CloudT::Scalar>::value, "PointCloud and KDTree have to have same internal type!" );
        static_assert( _CloudT::CloudT::Options & Eigen::RowMajor ,"PointCloud is assumed to be RowMajor for quick KdTree copy. You might need to change this part to point-wise copy" );

        KDTreeDataSetType dataset( new MxScalarT[cloud.getNPoints() * 3], cloud.getNPoints(), 3 );
        memcpy( dataset[0], cloud.getPoint(0).data(), cloud.getNPoints() * sizeof(MxScalarT) * 3 );
        kdtree.reset( new KDTreeType(dataset, flann::KDTreeSingleIndexParams(15)) );

        // initialize kdtree
        kdtree->buildIndex();

        return kdtree;
    } //...buildKdTree()

    /**
     * \tparam      _CloudT      Concept: \ref ColoredPointCloud.
     * \param[in]  kdtree       Target cloud to perform the lookup in.
     * \param[in]  cloud        Source cloud to get lookup locations from.
     * \param[in]  pid          Id of point in \p cloud, of which we are looking for neighbours of.
     * \param[out] indicesList  Output neighbour indices referring to point ids in \p kdtree.
     * \param[out] distsList    Output squared distances.
     * \param[in]  radius       Maximum distance between points in cloud and their neighbours in kdtree.
     * \param[in]  searchParams Pre-allocated flann configuration.
     */
    template <typename _CloudT>
    inline int knnSearch( const KDTreeType         & kdtree
                        , const _CloudT            & cloud
                        , const size_t               pid
                        ,       FlannIndicesT      & indicesList
                        ,       FlannDistsT        & distsList
                        , const size_t               K
                        , const flann::SearchParams& searchParams
                        )
    {
        typedef typename KDTreeType::ElementType ElementType;
        typedef typename _CloudT::Scalar         Scalar;

        static_assert( std::is_same<ElementType, typename _CloudT::Scalar>::value, "PointCloud and KDTree have to have same internal type!" );
        static_assert( _CloudT::CloudT::Options & Eigen::RowMajor ,"PointCloud is assumed to be RowMajor for quick KdTree copy. You might need to change this part to point-wise copy" );

        return kdtree.knnSearch( flann::Matrix<ElementType>( const_cast<Scalar*>( cloud.getPoint(pid).data() ), 1, 3), // !!! deconstcasting...and trusting flann...
                                 indicesList, distsList, K, searchParams );
    }

    /**
     *  \tparam      _CloudT      Concept: \ref ColoredPointCloud.
     *  \param[in]  kdtree       Target cloud to perform the lookup in.
     *  \param[in]  cloud        Source cloud to get lookup locations from.
     *  \param[in]  pid          Id of point in \p cloud, of which we are looking for neighbours of.
     *  \param[out] indicesList  Output neighbour indices referring to point ids in \p kdtree.
     *  \param[out] distsList    Output squared distances.
     *  \param[in]  radius       Maximum distance between points in cloud and their neighbours in kdtree.
     *  \param[in]  searchParams Pre-allocated flann configuration.
     */
    template <typename _CloudT>
    inline int radiusSearch( const KDTreeType               & kdtree
                           , const _CloudT                  & cloud
                           , const size_t                     pid
                           ,       FlannIndicesT            & indicesList
                           ,       FlannDistsT              & distsList
                           , const typename _CloudT::Scalar   radius
                           , const flann::SearchParams      & searchParams
                           )
    {
        typedef typename KDTreeType::ElementType ElementType;
        typedef typename _CloudT::Scalar         Scalar;

        static_assert( std::is_same<ElementType, typename _CloudT::Scalar>::value, "PointCloud and KDTree have to have same internal type!" );
        static_assert( _CloudT::CloudT::Options & Eigen::RowMajor ,"PointCloud is assumed to be RowMajor for quick KdTree copy. You might need to change this part to point-wise copy" );

        return kdtree.radiusSearch( flann::Matrix<ElementType>(const_cast<Scalar*>(cloud.getPoint(pid).data()),1,3), // !!! deconstcasting...and trusting flann...
                                    indicesList, distsList, radius, searchParams );
    }

    template <typename _PointT>
    inline int radiusSearch( const KDTreeType               & kdtree
                           , const _PointT                  & point
                           ,       FlannIndicesT            & indicesList
                           ,       FlannDistsT              & distsList
                           , const typename _PointT::Scalar   radius
                           , const flann::SearchParams      & searchParams
                           )
    {
        typedef typename KDTreeType::ElementType ElementType;
        typedef typename _PointT::Scalar         Scalar;

        static_assert( std::is_same<ElementType, typename _PointT::Scalar>::value, "Point and KDTree have to have same internal type!" );

        return kdtree.radiusSearch( flann::Matrix<ElementType>(const_cast<Scalar*>(point.data()),1,3), // !!! deconstcasting...and trusting flann...
                                    indicesList, distsList, radius, searchParams );
    }

    /**
     *  \tparam     _CloudT      Concept: \ref ColoredPointCloud.
     *  \param[in]  kdtree       Target cloud to perform the lookup in.
     *  \param[in]  cloud        Source cloud to get lookup locations from.
     *  \param[out] indicesList  Output neighbour indices referring to point ids in \p kdtree.
     *  \param[out] distsList    Output squared distances.
     *  \param[in]  radius       Maximum distance between points in cloud and their neighbours in kdtree.
     *  \param[in]  searchParams Pre-allocated flann configuration.
     */
    template <typename _CloudT>
    inline int radiusSearchCloud( const KDTreeType               & kdtree
                                , const _CloudT                  & cloud
                                ,       FlannIndicesT            & indicesList
                                ,       FlannDistsT              & distsList
                                , const typename _CloudT::Scalar   radius
                                , const flann::SearchParams      & searchParams
                                )
    {
        typedef typename KDTreeType::ElementType ElementType;
        typedef typename _CloudT::Scalar         Scalar;

        static_assert( std::is_same<ElementType, typename _CloudT::Scalar>::value, "PointCloud and KDTree have to have same internal type!" );
        static_assert( _CloudT::CloudT::Options & Eigen::RowMajor ,"PointCloud is assumed to be RowMajor for quick KdTree copy. You might need to change this part to point-wise copy" );

        return kdtree.radiusSearch( flann::Matrix<ElementType>(const_cast<Scalar*>(cloud.getPoint(0).data()),cloud.getNPoints(),3), // !!! deconstcasting...and trusting flann...
                                    indicesList, distsList, radius, searchParams );
    }

    /**
     * \tparam      _CloudT      Concept: \ref ColoredPointCloud.
     * \param[in]  kdtree       Target cloud to perform the lookup in.
     * \param[in]  cloud        Source cloud to get lookup locations from.
     * \param[in]  pid          Id of point in \p cloud, of which we are looking for neighbours of.
     * \param[out] indicesList  Output neighbour indices referring to point ids in \p kdtree.
     * \param[out] distsList    Output squared distances.
     * \param[in]  radius       Maximum distance between points in cloud and their neighbours in kdtree.
     * \param[in]  searchParams Pre-allocated flann configuration.
     */
    template <typename _CloudT>
    inline int knnSearchCloud( const KDTreeType               &kdtree
                             , const _CloudT                  &cloud
                             ,       FlannIndicesT            &indicesList
                             ,       FlannDistsT              &distsList
                             , const size_t                    K
                             , const typename _CloudT::Scalar  radius
                             , const flann::SearchParams      &searchParams
                             )
    {
        typedef typename KDTreeType::ElementType ElementType;
        typedef typename _CloudT::Scalar         Scalar;

        static_assert( std::is_same<ElementType, typename _CloudT::Scalar>::value, "PointCloud and KDTree have to have same internal type!" );
        static_assert( _CloudT::CloudT::Options & Eigen::RowMajor ,"PointCloud is assumed to be RowMajor for quick KdTree copy. You might need to change this part to point-wise copy" );
        indicesList.clear();
        distsList  .clear();

        if ( radius > 0. )
        {
            const Scalar radiusSqr( radius * radius );

            FlannIndicesT tmpInds;
            FlannDistsT   tmpDists;
            int res = kdtree.knnSearch( flann::Matrix<ElementType>( const_cast<Scalar*>( cloud.getPoint(0).data() ), cloud.getNPoints(), 3), // !!! deconstcasting...and trusting flann...
                                                    tmpInds, tmpDists, K, searchParams );
            indicesList.resize( tmpInds .size() );
            distsList  .resize( tmpDists.size() );
            for ( size_t i = 0; i != tmpInds.size(); ++i )
            {
                indicesList[i].reserve( tmpInds [i].size() );
                distsList  [i].reserve( tmpDists[i].size() );
                for ( size_t j = 0; j != tmpInds[i].size(); ++j )
                {
                    if ( tmpDists[i][j] < radiusSqr )
                    {
                        indicesList[i].push_back( tmpInds [i][j] );
                        distsList  [i].push_back( tmpDists[i][j] );
                    } //...if inside
                } //...for each neighbour
            } //...for each point
            return res;
        } //...if radius limited
        else
        {
            return kdtree.knnSearch( flann::Matrix<ElementType>( const_cast<Scalar*>( cloud.getPoint(0).data() ), cloud.getNPoints(), 3), // !!! deconstcasting...and trusting flann...
                                        indicesList, distsList, K, searchParams );
        } //...if no radius limit
    } //...knnSearchCloud()

    template <typename _PointT>
    size_t getNNtopK( const KDTreeType                           * kdtree
                    , const _PointT                              & pnt
                    ,       FlannIndicesT                        & outIndicesList
                    ,       FlannDistsT                          & outDistSqrs
                    , const int                                    K
                    )
    {
        typedef KDTreeType::DistanceType Scalar;
        static_assert( std::is_same<Scalar, typename _PointT::Scalar>::value, "[getNNtopK] KDTreeType and _PointT have to have the same float type" );
        static const flann::SearchParams searchParams( /* checks: */ 32, /* eps: */ 0.0, /* sorted: */ true );

        //FlannIndicesT indicesList;
        //FlannDistsT   distSqrsList;

        // warning, deconst cast!
        kdtree->knnSearch( flann::Matrix<Scalar>(const_cast<Scalar*>(pnt.data()),1,3), outIndicesList, outDistSqrs, K, searchParams );
        //filterByRadius( outIndices, outDistSqrs, indicesList, distSqrsList, maxRadiusSqr );


        return outIndicesList.size();
    } //...getNNtopK
} //...ns Soup

#endif // KDTREEUTIL_HPP

