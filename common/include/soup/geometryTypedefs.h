#ifndef SOUP_GEOMETRY_TYPEDEFS_H
#define SOUP_GEOMETRY_TYPEDEFS_H

#include "tracking/common/eigen.h"

namespace Soup {
namespace geometry {

typedef float                                      Scalar;      //!< THE scalar definition.
typedef Eigen::Transform<Scalar,3, Eigen::Affine>  TransformT;
typedef Eigen::Matrix   <Scalar,3,1>               Vector3;
typedef Eigen::Matrix   <Scalar,3,3>               Matrix3;
typedef Eigen::Matrix   <Scalar,4,4>               Matrix4;
typedef Eigen::Matrix   <Scalar,4,1>               Vector4;
typedef Eigen::Translation<Scalar,3>               TranslationT;

template <typename _Scalar>
struct HelperTypes {
    typedef _Scalar                                     Scalar;
    typedef Eigen::Matrix     <Scalar,3,Eigen::Dynamic> Matrix3X;
    typedef Eigen::Matrix     <Scalar,1,3>              RowVector3;
    typedef Eigen::Translation<Scalar,3>                Translation;
    typedef Eigen::Transform  <Scalar,3, Eigen::Affine> Transform;
    typedef Eigen::Matrix     <Scalar,Eigen::Dynamic,3> MatrixX3;
    typedef Eigen::AngleAxis  <Scalar>                  AngleAxisT;
    using Vector3 = Soup::geometry::Vector3;
    using Vector4 = Soup::geometry::Vector4;
    using Matrix3 = Soup::geometry::Matrix3;
    using Matrix4 = Soup::geometry::Matrix4;
}; //...struct HelperTypes
} //...ns geometry
} //...Soup

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Soup::geometry::Vector4)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Soup::geometry::Matrix3)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Soup::geometry::Matrix4)

#endif // SOUP_GEOMETRY_TYPEDEFS_H
