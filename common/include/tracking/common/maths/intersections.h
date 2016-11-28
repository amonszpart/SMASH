#ifndef SOUP_MATHS_INTERSECTIONS_H
#define SOUP_MATHS_INTERSECTIONS_H

#include "Eigen/Core"

namespace Soup
{
    template <typename _Scalar>
    bool lineLineIntersection( const Eigen::Matrix<_Scalar,3,1>& p0
                             , const Eigen::Matrix<_Scalar,3,1>& v0
                             , const Eigen::Matrix<_Scalar,3,1>& p1
                             , const Eigen::Matrix<_Scalar,3,1>& v1
                             ,       Eigen::Matrix<_Scalar,3,1>* intersectionPoint
                             , const _Scalar                     distThreshold
                             , const _Scalar                     eps
                             );
} //...ns Soup

#include "soup/maths/impl/intersections.hpp"

#endif // SOUP_MATHS_INTERSECTIONS_H
