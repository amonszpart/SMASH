/*
 * Created by: Aron Monszpart (aron.monszpart@gmail.com)
 * On: 8/6/2015
 *
 * Property of Adobe Inc.
*/

#include "soup/geometryTypedefs.h"
#include "soup/pointCloudEigen.h"
#include "soup/impl/pointCloudEigen.hpp"

namespace Soup {
  using namespace geometry;

  template class PointCloud<float>;
  template class PointCloud<double>;
  template class ColoredPointCloud<float>;
  template class ColoredPointCloud<double>;
#if 0
  template
    void ColoredPointCloud<double>::fromMesh( const QUASAR::Mesh& mesh, typename CloudT::Ptr& cloud, typename CloudT::Ptr& colors, typename CloudT::Ptr& normals );
    template
    void ColoredPointCloud<float >::fromMesh( const QUASAR::Mesh& mesh, typename CloudT::Ptr& cloud, typename CloudT::Ptr& colors, typename CloudT::Ptr& normals );

    template
    void PointCloud<double>::fromMesh( const QUASAR::Mesh& mesh, PointCloud<double>& cloud, PointCloud<double>& colors, PointCloud<double>& normals );
    template
    void PointCloud<float >::fromMesh( const QUASAR::Mesh& mesh, PointCloud<float >& cloud, PointCloud<float >& colors, PointCloud<float>& normals );

    template
    void ColoredPointCloud<double>::fromMesh( const QUASAR::Mesh& mesh, PtrT& cloud );
    template
    void ColoredPointCloud<float >::fromMesh( const QUASAR::Mesh& mesh, PtrT& cloud );

      template
    void PointCloud<Scalar>::toList( const PointCloud<Scalar>& cloud0, typename QUASAR::Mesh::VerticesT& out );
    template
    void PointCloud<Scalar>::toList( const PointCloud<Scalar>& cloud0, typename QUASAR::Mesh::ColorsT& out );

    template
    void ColoredPointCloud<float>::writeMesh<QUASAR::Mesh>( const ConstPtrT& cloud, const std::string& path );
    template
    void ColoredPointCloud<double>::writeMesh<QUASAR::Mesh>( const ConstPtrT& cloud, const std::string& path );

    template
    typename ColoredPointCloud<Soup::geometry::Scalar>::PtrT ColoredPointCloud<Soup::geometry::Scalar>::read<QUASAR::Mesh>( const std::string& path );
#endif

  template
  void randomCloud( PointCloud<float>& points, const size_t nPoints, const Eigen::Matrix<float,3,1>& centroid );
  template
  void randomCloud( PointCloud<double>& points, const size_t nPoints, const Eigen::Matrix<double,3,1>& centroid );
  template
  void PointCloud<Scalar>::transformCloud( const Eigen::Transform <Scalar, 3, 2, 0> &transform,
      PointCloud<Scalar>& outPoints ) const;
  template
  void ColoredPointCloud<Scalar>::transformCloud( const Eigen::Transform <Scalar, 3, 2, 0> &transform,
      ColoredPointCloud<Scalar>& outPoints ) const;
  //template ColoredPointCloud<float>::RowVector3 ColoredPointCloud<float>::getCentroid() const;
  //template ColoredPointCloud<double>::RowVector3 ColoredPointCloud<double>::getCentroid() const;
//    template
//    void transformCloud( const Eigen::MatrixBase<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor, Eigen::Dynamic, 3> >& points,
//                         const Eigen::Transform <Scalar, 3, 2, 0> &transform,
//                         const Eigen::MatrixBase<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, Eigen::RowMajor, Eigen::Dynamic, 3>> & outPoints );

//    template
//    void PointCloud<Scalar>::fromMesh( const QUASAR::Mesh& mesh, Ptr& cloud, Ptr& colors );
}
