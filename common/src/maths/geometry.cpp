/*
 * Created by: Aron Monszpart (aron.monszpart@gmail.com)
 * On: 7/23/2015
 *
 * Property of Adobe Inc.
*/


#include "tracking/common/maths/impl/geometry.hpp"
#include "tracking/common/maths/geometry.h"
#include "soup/geometryTypedefs.h"

namespace maths {
  template float
  point2Line(Eigen::MatrixBase<Eigen::Vector3f> const& point,
             Eigen::MatrixBase<Eigen::Vector3f> const& lineStart,
             Eigen::MatrixBase<Eigen::Vector3f> const& lineDir);

  template double
  point2Line(Eigen::MatrixBase<Eigen::Vector3d> const& point,
             Eigen::MatrixBase<Eigen::Vector3d> const& lineStart,
             Eigen::MatrixBase<Eigen::Vector3d> const& lineDir);

//    /*! \brief Calculates angle of two vectors in radians a more safe way, than acos. */
//    template <typename DerivedA, typename DerivedB>
//    typename DerivedA::Scalar
//    angleInRad( const DerivedA& v1, const DerivedB& v2 );

//    /*! \brief Calculates the absolute angle of two vectors in radians a more safe way, than acos. */
//    template <typename DerivedA, typename DerivedB>
//    typename DerivedA::Scalar
//    absAngleInRad( const DerivedA& v1, const DerivedB& v2 );

  template float
  angleToParallel(Eigen::Vector3f const& v1, Eigen::Vector3f const& v2);
  template double
  angleToParallel(Eigen::Vector3d const& v1, Eigen::Vector3d const& v2);

  template void
  flippedAdd(Eigen::Vector3f& v1, Eigen::Vector3f const& v2);
  template void
  flippedAdd(Eigen::Vector3d& v1, Eigen::Vector3d const& v2);

} //...ns Soup
