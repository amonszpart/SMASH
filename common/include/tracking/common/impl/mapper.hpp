#ifndef TV_MAPPER_HPP
#define TV_MAPPER_HPP

#include "tracking/common/mapper.h"

namespace tracking
{
    template <typename _Vector2>
    inline float Mapper::getDepthValue( const cv::Mat& depth, const _Vector2& pnt2) {
        if ( depth.type() != CV_32FC1 )
            throw new Mapper_DepthMapNot32FC1Exception(".at<float> won't work this way!");
        return depth.at<float>( pnt2(1),pnt2(0) );
    }

    template <typename _Vector2>
    inline float Mapper::getClosestDepthValue( const cv::Mat& depth, const _Vector2& pnt2, float minValue, int maxRange )
    {
        if ( depth.type() != CV_32FC1 )
            throw new Mapper_DepthMapNot32FC1Exception(".at<float> won't work this way!");

        float d = Mapper::getDepthValue( depth, pnt2 );
        if ( d > minValue )
            return d;

        int halfRange = std::max( maxRange / 2, 1 );
        cv::Point2i start( std::max( static_cast<int>(pnt2(0)) - halfRange, 0          ), std::max(static_cast<int>(pnt2(1))-halfRange,         0) );
        cv::Point2i stop ( std::max( static_cast<int>(pnt2(0)) + halfRange, depth.cols ), std::min(static_cast<int>(pnt2(1))+halfRange,depth.rows) );
        float minDist = std::numeric_limits<float>::max(), tmpDepth, tmpDist;
        _Vector2 p( start.x, start.y );
        for ( ; p.y() < stop.y; ++p.y() )
            for ( ; p.x() < stop.x; ++p.x() )
            {
                tmpDepth = Mapper::getDepthValue( depth, p );
                if ( tmpDepth > minValue )
                {
                    tmpDist = (p - pnt2).norm();
                    if ( tmpDist < minDist )
                    {
                        minDist = tmpDist;
                        d       = tmpDepth;
                    }
                }
            }
        if ( minDist < std::numeric_limits<float>::max() )
            return d;
        else
            return Mapper::getDepthValue(depth,pnt2);
    } //...getClosestDepthValue()

    template <typename _Vector2>
    inline Mapper::Vector3 Mapper::to3D( const _Vector2& pnt2, const Scalar depth, const int height ) const
    {
        static_assert( _Vector2::RowsAtCompileTime + _Vector2::ColsAtCompileTime == 3 , "Need Vector2 in Mapper::to3D" );
        Vector3 pnt3;
        if ( _invertY )
            pnt3 << pnt2(0), height - pnt2(1) - 1, 1.;
        else
            pnt3 << pnt2(0), pnt2(1), 1.;
        return _intrinsics.inverse() * pnt3 * depth;
    }

    template <typename _Vector2>
    inline Mapper::Vector3 Mapper::to3D( const _Vector2& pnt2, const Scalar depth ) const
    {
        static_assert( _Vector2::RowsAtCompileTime + _Vector2::ColsAtCompileTime == 3 , "Need Vector2 in Mapper::to3D" );
        return _intrinsics.inverse() * (Vector3() << pnt2(0), pnt2(1), 1.).finished() * depth;
    } //...to3D

} //...ns tracking

#endif //...TV_MAPPER_HPP
