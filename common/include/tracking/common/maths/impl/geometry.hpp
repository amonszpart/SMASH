#ifndef TRACKING_COMMON_GEOMETRY_HPP
#define TRACKING_COMMON_GEOMETRY_HPP

#include "tracking/common/eigen.h"
#include <limits>
#include <cmath>        // atan2
#include <algorithm>    // min

namespace maths {
    /** \brief Implements \f$ d = (|(x_2-x_1) \times (x_1-x_0)|)/(|x_2-x_1|) \f$
     */
    template <typename _DerivedAT, typename _DerivedBT, typename _DerivedCT>
    inline auto
    point2Line(Eigen::MatrixBase <_DerivedAT> const& point, Eigen::MatrixBase <_DerivedBT> const& lineStart,
               Eigen::MatrixBase <_DerivedCT> const& lineDir) -> typename _DerivedAT::Scalar {
        return lineDir.cross(lineStart - point).norm() / lineDir.norm();
    } //...point2Line()

    /*!
     * \tparam DerivedA  Concept: Eigen::Matrix<Scalar,3,1>.
     * \tparam DerivedB  Concept: Eigen::Matrix<Scalar,3,1>.
     */
    template <typename DerivedA, typename DerivedB>
    inline typename DerivedA::Scalar
    angleInRad( const DerivedA& v1, const DerivedB& v2 )
    {
        typedef typename DerivedB::Scalar Scalar; // float or double

        Scalar angle = std::atan2( v1.cross(v2).norm(), v1.dot(v2) );
        return ( (angle != angle) ? Scalar(0.)  // fix nan
                                  : angle);
    } //...angleInRad

    /*!
     * \tparam DerivedA  Concept: Eigen::Matrix<Scalar,3,1>.
     * \tparam DerivedB  Concept: Eigen::Matrix<Scalar,3,1>.
     */
    template <typename DerivedA, typename DerivedB>
    inline typename DerivedA::Scalar
    absAngleInRad( const DerivedA& v1, const DerivedB& v2 )
    {
        return std::abs( angleInRad(v1,v2) );
    } //...absAngleInRad

    /*!
     * \tparam DerivedA  Concept: Eigen::Matrix<Scalar,3,1>.
     * \tparam DerivedB  Concept: Eigen::Matrix<Scalar,3,1>.
     */
    template <typename DerivedA, typename DerivedB>
    inline typename DerivedA::Scalar
    angleToParallel( const DerivedA& v1, const DerivedB& v2 )
    {
        typedef typename DerivedA::Scalar Scalar; // float or double

        Scalar angle = absAngleInRad(v1,v2);
        while ( angle > M_PI )
            angle -= M_PI;
        return std::min( angle, Scalar(M_PI) - angle );
    } //...angleToParallel

    /*!
     * Handy routine to accumulate directions and take their average.
     *
     * \tparam DerivedA  Concept: Eigen::Matrix<Scalar,3,1>.
     * \tparam DerivedB  Concept: Eigen::Matrix<Scalar,3,1>.
     *
     * \param[in,out] v1 Vector to add \p v2 to.
     * \param[in]     v2 Vector to add to \p v1, if pointing in the same direction, OR subtract, if not.
     */
    template <typename DerivedA, typename DerivedB>
    inline void
    flippedAdd( DerivedA& v1, const DerivedB& v2 )
    {
        if ( v1.dot(v2) < 0. )  v1 -= v2;
        else                    v1 += v2;
    } //...flippedAdd

} //...namespace Soup

#endif // TRACKING_COMMON_GEOMETRY_HPP

