#ifndef TRACKING_COMMON_GEOMETRY_H
#define TRACKING_COMMON_GEOMETRY_H

namespace maths {
    /*!
     * \brief Measures point to line distance.
     */
    template <typename _DerivedAT, typename _DerivedBT, typename _DerivedCT>
    inline auto
    point2Line(Eigen::MatrixBase <_DerivedAT> const& point, Eigen::MatrixBase <_DerivedBT> const& lineStart,
               Eigen::MatrixBase <_DerivedCT> const& lineDir) -> typename _DerivedAT::Scalar;

    /*! \brief Calculates angle of two vectors in radians a more safe way, than acos. */
    template <typename DerivedA, typename DerivedB>
    typename DerivedA::Scalar
    angleInRad( const DerivedA& v1, const DerivedB& v2 );

    /*! \brief Calculates the absolute angle of two vectors in radians a more safe way, than acos. */
    template <typename DerivedA, typename DerivedB>
    typename DerivedA::Scalar
    absAngleInRad( const DerivedA& v1, const DerivedB& v2 );

    /*! \return Smallest angle in radians to parallel relationship. */
    template <typename DerivedA, typename DerivedB>
    typename DerivedA::Scalar
    angleToParallel( const DerivedA& v1, const DerivedB& v2 );

    /*! \brief Adds \p v2 to \p v1, but flips it v2 first, if they are not pointing in the same direction. */
    template <typename DerivedA, typename DerivedB>
    void
    flippedAdd( DerivedA& v1, const DerivedB& v2 );

} //...namespace maths

//#include "soup/maths/impl/geometry.hpp"

#endif // TRACKING_COMMON_GEOMETRY_H

