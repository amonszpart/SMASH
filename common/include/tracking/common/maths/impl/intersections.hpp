#ifndef SOUP_MATHS_INTERSECTIONS_HPP
#define SOUP_MATHS_INTERSECTIONS_HPP

#include "Eigen/Core"
#include <iostream>

namespace Soup
{
    /*!
     * \brief Estimates intersection of two lines defined by points and directions.
     *
     * \param[in] p0            Origin of first line.
     * \param[in] v0            Direction of first line.
     * \param[in] p1            Origin of second line.
     * \param[in] v1            Direction of second line.
     * \param[in] distThreshold Controls, when the closest point of the two lines is close enough.
     * \param[in] eps           Controls, when the determinant is 0, i.e. the two vectors are not intersecting.
     * \return  false, if no intersection. Otherwise, true.
     */
    template <typename _Scalar>
    inline bool lineLineIntersection( const Eigen::Matrix<_Scalar,3,1>& p0
                                    , const Eigen::Matrix<_Scalar,3,1>& v0
                                    , const Eigen::Matrix<_Scalar,3,1>& p1
                                    , const Eigen::Matrix<_Scalar,3,1>& v1
                                    ,       Eigen::Matrix<_Scalar,3,1>* intersectionPointArg
                                    , const _Scalar                     distThreshold
                                    , const _Scalar                     eps
                                    )
    {
        //typedef Eigen::Matrix<_Scalar,3,1> Vector3;
        typedef Eigen::Matrix<_Scalar,3,3> Matrix3;

        // http://www.realtimerendering.com/intersections.html#I304
        // t1 = Determinant{(o2-o1),d2,d1 X d2} / ||d1 X d2||^2
        // t2 = Determinant{(o2-o1),d1,d1 X d2} / ||d1 X d2||^2

        auto    cross       = v0.cross( v1 );
        _Scalar squaredNorm = cross.squaredNorm();
        if ( squaredNorm < eps )
        {
            //std::cout << "[" << __func__ << "]: sqNorm: " << squaredNorm << std::endl;
            return false;
        }

        auto i0 = p0 + v0 * (Matrix3() << p1 - p0, v1, cross).finished().determinant() / squaredNorm;
        auto i1 = p1 + v1 * (Matrix3() << p1 - p0, v0, cross).finished().determinant() / squaredNorm;

        if ( (i1-i0).norm() < distThreshold )
        {
            if ( intersectionPointArg )
                *intersectionPointArg = (i0 + i1) / _Scalar(2.);
            return true;
        }
        else
        {
            //std::cout << "norm: " << (i1-i0).norm() << std::endl;
            return false;
        }

#if 0
        Vector3  tmp,
                &intersectionPoint = intersectionPointArg ? *intersectionPointArg
                                                          : tmp;
        Eigen::ParametrizedLine<_Scalar,3> l0(p0,v0.normalized());
        Eigen::Hyperplane<_Scalar,3>       h1( Eigen::Hyperplane<_Scalar,3>::Through(p1,p1+v1) );


//        intersectionPoint = Eigen::ParametrizedLine<_Scalar,3>(p0,v0).intersectionPoint(
//                    Eigen::Hyperplane<_Scalar,3>::Through(p1,p1+v1) );
        //intersectionPoint = l0.intersectionPoint( h1 );
        std::cout << "distline: " << l0.distance(intersectionPoint) << std::endl;
        std::cout << "distplane: " << h1.absDistance( intersectionPoint)  << std::endl;
        std::cout << "l0: " << l0.origin().transpose() << ", " << l0.direction().transpose() << std::endl;
        std::cout << "h1: " << h1.coeffs().transpose() << std::endl;
        std::cout << "intersect: " << intersectionPoint.transpose() << std::endl;
        std::cout << "p0,v0: " << p0.transpose() << ", " << v0.transpose() << std::endl;
        std::cout << "p1,v1: " << p1.transpose() << ", " << v1.transpose() << std::endl; fflush(stdout);
#warning FIX
        if ( std::abs(l0.distance(intersectionPoint)) > 0.001
             || h1.absDistance( intersectionPoint) > 0.001 )
        {
            return false;
        }


        //L=(x2-x1)x(x4-x3)
        // (vector product) is vector perpendicular to plane in which both directing vectors (x2-x1) and (x4-x3) lie.
        // Now, if lines intersect, vector (x3-x1) is also perpendicular to that plane, which you may test by scalar product with perpendicular.
        // if L*(x3-x1) is 0 then lines intersect.

        //intersectionPoint = Eigen::ParametrizedLine<_Scalar,3>(p0,v0).intersectionPoint(
        //            Eigen::Hyperplane<_Scalar,3>(Eigen::ParametrizedLine<_Scalar,3>(p1,v1)) );


        // nans, if lines don't intersect
        return (intersectionPoint.array() == intersectionPoint.array()).all();
#endif

    } //...lineLineIntersection()
} //...ns Soup

#endif // SOUP_MATHS_INTERSECTIONS_HPP

