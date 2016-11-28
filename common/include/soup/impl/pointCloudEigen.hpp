#ifndef SOUP_POINTCLOUDEIGEN_HPP
#define SOUP_POINTCLOUDEIGEN_HPP

#include "soup/pointCloudEigen.h"
#include "tracking/common/util/util.h"       // randf()
//#include "soup/typedefs.h"                  // VecTypeMap

namespace Soup {
  using tracking::randf;
  /*!
   * \tparam _CloudT  Concept: geometry::MatrixX3
   * \tparam _Vector3 Concept: geometry::Vector3
   */
  template <typename _CloudT, typename _Vector3>
  inline void randomCloud( _CloudT& points, const size_t nPoints, const _Vector3& centroid )
  {
      points.resize( nPoints, 3 );
      for ( int r = 0; r != points.rows(); ++r )
      {
          points.row(r) = _Vector3( centroid(0) - 0.1 + randf(0.2),
              centroid(1) - 0.1 + randf(0.2),
              centroid(2) - 0.1 + randf(0.2) );
      }
  } //...randomCloud()

  template <typename _Scalar>
  template <typename _TransformT>
  inline void PointCloud<_Scalar>::transformCloud( const _TransformT& transform, PointCloud<_Scalar>& outPoints ) const
  {
      // this method was designed for points in rows
      eigen_assert( PointCloud<_Scalar>::RowsAtCompileTime == -1 ||
                    PointCloud<_Scalar>::RowsAtCompileTime > PointCloud<_Scalar>::ColsAtCompileTime );

      outPoints = (transform * this->transpose()).transpose();
  }

  template <typename _Scalar>
  template <typename _TransformT>
  inline void ColoredPointCloud<_Scalar>::transformCloud( const _TransformT& transform, ColoredPointCloud<_Scalar>& outPoints ) const
  {
      // this method was designed for points in rows
      eigen_assert( PointCloud<_Scalar>::RowsAtCompileTime == -1 ||
                    PointCloud<_Scalar>::RowsAtCompileTime > PointCloud<_Scalar>::ColsAtCompileTime );

      this->getPoints().transformCloud( transform, outPoints.getPoints() );
      outPoints.getColors() = this->getColors();
      this->getNormals().transformCloud( transform, outPoints.getNormals() );
  }
#if 0
  template <typename _Scalar>
  template <typename _MeshT>
  void ColoredPointCloud<_Scalar>::fromMesh( const _MeshT& mesh, typename CloudT::Ptr& cloud, typename CloudT::Ptr& colors, typename CloudT::Ptr& normals )
  {
      //PointCloud<_Scalar>
      cloud  .reset( new PointCloud<_Scalar>(mesh.getVertices().size(), /*PointCloud<_Scalar>::Dim*/ 3) );
      colors .reset( new PointCloud<_Scalar>(mesh.getColors  ().size(), /*PointCloud<_Scalar>::Dim*/ 3) );
      normals.reset( new PointCloud<_Scalar>(mesh.getNormals ().size(), /*PointCloud<_Scalar>::Dim*/ 3) );

      CloudT::fromMesh( mesh, *cloud, *colors, *normals );

//        if ( (cloud->getNPoints() != colors->getNPoints()) && (colors->getNPoints() > 0) )
//            throw new PointCloudException("Mesh needs to have the same amount of vertices and colors!");
//        *cloud = Eigen::Map< const Eigen::Matrix<ScalarType, Eigen::Dynamic, 3, Eigen::RowMajor> >(
//                     mesh.getVertices()[0].data(), mesh.getVertices().size(), 3 ).cast<_Scalar>();
//        for ( size_t i = 0; i != mesh.getColors().size(); ++i )
//            colors->setPoint( i, Eigen::Map< const Eigen::Matrix<typename _MeshT::Scalar,3,1> >(mesh.getColor(i).data()) );
  }

  template <typename _Scalar>
  template <typename _MeshT>
  void PointCloud<_Scalar>::fromMesh( const _MeshT& mesh, PointCloud<_Scalar>& cloud, PointCloud<_Scalar>& colors, PointCloud<_Scalar>& normals )
  {
      //PointCloud<_Scalar>
      cloud .resize( mesh.getVertices().size(), static_cast<int>(PointCloud<_Scalar>::Dim) );
      colors.resize( mesh.getColors  ().size(), static_cast<int>(PointCloud<_Scalar>::Dim) );
      normals.resize( mesh.getNormals().size(), static_cast<int>(PointCloud<_Scalar>::Dim) );

      if ( cloud.getNPoints() != colors.getNPoints() && colors.getNPoints() > 0 )
          throw new PointCloudException("Mesh needs to have the same amount of vertices and colors!");
      if ( cloud.getNPoints() != normals.getNPoints() && normals.getNPoints() > 0 )
          throw new PointCloudException("Mesh needs to have the same amount of vertices and normals!");

      cloud = Eigen::Map< const Eigen::Matrix<ScalarType, Eigen::Dynamic, 3, Eigen::RowMajor> >(
          mesh.getVertices()[0].data(), mesh.getVertices().size(), 3 ).cast<_Scalar>();
      colors = Eigen::Map< const Eigen::Matrix<ScalarType, Eigen::Dynamic, 3, Eigen::RowMajor> >(
          mesh.getColors()[0].data(), mesh.getColors().size(), 3 ).cast<_Scalar>();
      normals = Eigen::Map< const Eigen::Matrix<ScalarType, Eigen::Dynamic, 3, Eigen::RowMajor> >(
          mesh.getNormals()[0].data(), mesh.getNormals().size(), 3 ).cast<_Scalar>();
//        for ( size_t i = 0; i != mesh.getVertices().size(); ++i )
//            cloud.setPoint ( i, Eigen::Map< const Eigen::Matrix<typename _MeshT::Scalar,3,1> >(mesh.getVertex(i).data()) );
//        for ( size_t i = 0; i != mesh.getColors().size(); ++i )
//            colors.setPoint( i, Eigen::Map< const Eigen::Matrix<typename _MeshT::Scalar,3,1> >(mesh.getColor(i).data()) );
  }

  template <typename _Scalar>
  template <typename _MeshT>
  void ColoredPointCloud<_Scalar>::writeMesh( const ConstPtrT& cloud, const std::string& name )
  {
      _MeshT m;
      CloudT::toList( cloud->getPoints (), m.getVertices() );
      CloudT::toList( cloud->getColors (), m.getColors  () );
      CloudT::toList( cloud->getNormals(), m.getNormals() );
      m.write( name.c_str() );
      std::cout << "[" << __func__ << "]: wrote to " << name << std::endl;
  } //...writeMesh()

  template <typename _Scalar>
  template <typename _MeshT>
  typename ColoredPointCloud<_Scalar>::PtrT ColoredPointCloud<_Scalar>::read( const std::string& path )
  {
      PtrT out;
      _MeshT mesh;
      mesh.read( path.c_str() );
      fromMesh( mesh, out );
      return out;
  } //...writeMesh()

  template <typename _Scalar>
  template <typename _ListType, typename _VecType>
  void PointCloud<_Scalar>::toList( const PointCloud<_Scalar>& cloud0, _ListType& out )
  {
      out.resize( cloud0.getNPoints() );
      for ( size_t row = 0; row != cloud0.getNPoints(); ++row )
      {
          VecTypeMap(out[row].data()) = cloud0.getPoint(row).template cast<ScalarType>();
      }
  } //...toList()
#endif

  template <typename _Scalar>
  void PointCloud<_Scalar>::cat( Ptr& out, const PointCloud<_Scalar>& a, const PointCloud<_Scalar>& b )
  {
      if ( (&a == &(*out)) || (&b == &(*out)) )
          throw new PointCloudCatException("output \"out\" can't be the same as input \"a\" or \"b\", use append for this purpose");

      out.reset( new PointCloud(a.getNPoints() + b.getNPoints(), int(PointCloud::Dim)) );
      cat( *out, a, b );
      //Eigen::Block<MyBase>( *out, a->getNPoints(), 0, b->getNPoints(), int(PointCloud::Dim) ) = *b;
      //Eigen::Block<MyBase>( *out,               0, 0, a->getNPoints(), int(PointCloud::Dim) ) = *a;
  }

  template <typename _Scalar>
  void PointCloud<_Scalar>::cat( PointCloud<_Scalar>& out, const PointCloud<_Scalar>& a, const PointCloud<_Scalar>& b )
  {
      if ( out.rows() != a.rows() + b.rows() )
          out.conservativeResize( a.rows() + b.rows(), Eigen::NoChange );
      out.block(              0, 0, a.getNPoints(), a.cols() ) = a;
      out.block( a.getNPoints(), 0, b.getNPoints(), b.cols() ) = b;
  }

  template <typename _Scalar>
  void PointCloud<_Scalar>::append( const ConstPtr& a )
  {
      this->append( *a );
  }

  template <typename _Scalar>
  void PointCloud<_Scalar>::append( const PointCloud<_Scalar>& a )
  {
      const size_t oldRows = this->rows();
      this->conservativeResize( this->rows() + a.rows(), Eigen::NoChange );
      this->block             ( oldRows, 0, this->rows(), this->cols() ) = a;
  }

  template <typename _Scalar>
  void ColoredPointCloud<_Scalar>::cat( PtrT& out, const ConstPtrT& a, const ConstPtrT& b )
  {
      if ( (a == out) || (b == out) )
          throw new ColoredPointCloudCatException("output \"out\" can't be the same as input \"a\" or \"b\", use append for this purpose");

      out.reset( new ColoredPointCloud(a->getNPoints() + b->getNPoints(), int(ColoredPointCloud::Dim)) );
      CloudT::cat( out->getPoints(), a->getPoints(), b->getPoints() );
      CloudT::cat( out->getColors(), a->getColors(), b->getColors() );
      CloudT::cat( out->getNormals(), a->getNormals(), b->getNormals() );
  }

  template <typename _Scalar>
  void ColoredPointCloud<_Scalar>::append( const ConstPtrT& a )
  {
      this->getPoints().append( a->getPoints() );
      this->getColors().append( a->getColors() );
      this->getNormals().append( a->getNormals() );
  }

  template <typename _Scalar>
  typename ColoredPointCloud<_Scalar>::RowVector3 ColoredPointCloud<_Scalar>::getCentroid() const {
      return this->getPoints().colwise().mean();
  }

} //...ns Soup

#endif // SOUP_POINTCLOUDEIGEN_HPP

