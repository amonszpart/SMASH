#ifndef SOUP_POINTCLOUDEIGEN_H
#define SOUP_POINTCLOUDEIGEN_H

#include "tracking/common/eigen.h"
#include "tracking/common/util/exception.h"
#include "soup/cloudTypedefs.h"
#include <vector>
#include <memory>

namespace Soup {
    template <typename _CloudT, typename _Vector3>
    void randomCloud( _CloudT& points, const size_t nPoints, const _Vector3& centroid = _Vector3::Ones() );

    //template <typename _DerivedT, typename _TransformT>
    //void transformCloud( const Eigen::MatrixBase<_DerivedT>& points, const _TransformT& transform, const Eigen::MatrixBase<_DerivedT> & outPoints );

//    template <typename _CloudT>
//    typename _CloudT::ConstRowXpr getPoint( const _CloudT &cloud, size_t i ) { return cloud.row(i); }

    template <typename _Scalar>
    class PointCloud : public Eigen::Matrix<_Scalar,Eigen::Dynamic,3, Eigen::RowMajor> {
            DEFINE_EXCEPTION(PointCloud)
            DEFINE_EXCEPTION(PointCloudCat)

        public:
            typedef _Scalar Scalar;
            enum EnumType : int { Dim = 3 };
            typedef Eigen::Matrix<_Scalar,Eigen::Dynamic,Dim, Eigen::RowMajor>  MyBase;
            typedef typename Eigen::Matrix<Scalar,3,1>                          Vector3;
            typedef std::shared_ptr<       PointCloud<_Scalar> > Ptr;
            typedef std::shared_ptr<       PointCloud<_Scalar> > PtrT;
            typedef std::shared_ptr< const PointCloud<_Scalar> > ConstPtr;
            typedef std::shared_ptr< const PointCloud<_Scalar> > ConstPtrT;

            PointCloud(void) : MyBase() {}

            // This constructor allows you to construct PointCloud from Eigen expressions
            template <typename OtherDerived>
            PointCloud(const Eigen::MatrixBase<OtherDerived>& other) : MyBase(other) {}

            // This method allows you to assign Eigen expressions to PointCloud
            template<typename OtherDerived>
            PointCloud& operator=(const Eigen::MatrixBase<OtherDerived>& other) { this->MyBase::operator=(other); return *this; }

            using MyBase::MyBase;

            inline typename MyBase::ConstRowXpr getPoint  ( size_t i ) const { return this->row(i); }
            inline typename MyBase::RowXpr      getPoint  ( size_t i )       { return this->row(i); }
            inline size_t                       getNPoints()           const { return this->rows(); }

            template <typename _DerivedT>
            inline void                         setPoint   ( size_t i, const Eigen::MatrixBase<_DerivedT>& pnt ) { this->row(i) = pnt.template cast<_Scalar>(); }

            template <typename _MeshT>
            static void                         fromMesh( const _MeshT& mesh, PointCloud& cloud, PointCloud& color, PointCloud& normals );

            //template <typename _ListType, typename _VecType = typename _ListType::value_type>
            //static void                         toList( const PointCloud<_Scalar>& cloud0, _ListType& out );
            //template <typename _ColorsListType, typename _ColorType = typename _ColorsListType::value_type>
            //static void                         toColorsList( const PointCloud<_Scalar>& cloud0, _ColorsListType& out );

            static void cat( Ptr& out, const PointCloud& a, const PointCloud& b );
            static void cat( PointCloud& out, const PointCloud& a, const PointCloud& b );
            void append( const ConstPtr& a );
            void append( const PointCloud& a );

            template <typename _TransformT>
            void transformCloud( const _TransformT& transform, PointCloud<_Scalar>& outPoints ) const;

            PointCloud( const size_t rows ) : MyBase( rows, static_cast<size_t>(Dim) ) {}
    }; // class PointCloud

    /*! Compound pointcloud type. */
    template <typename _Scalar>
    class ColoredPointCloud {
            DEFINE_EXCEPTION(ColoredPointCloudCat)
        public:
            typedef PointCloud<_Scalar>                        CloudT; //!< InnerType
            enum { NeedsToAlign = (sizeof(CloudT)%16)==0 };
            enum {           Dim =                             CloudT::Dim };
            typedef typename CloudT::Scalar                    Scalar;
            typedef typename CloudT::RowXpr                    PointT;
            typedef typename CloudT::ConstRowXpr               ConstPointT;
            typedef typename Eigen::Matrix<Scalar,1,3>         RowVector3;
            typedef typename CloudT::Vector3                   Vector3;


            typedef std::shared_ptr<       ColoredPointCloud > PtrT;
            typedef std::shared_ptr< const ColoredPointCloud > ConstPtrT;
            //struct ConstPtrsT : public std::vector<ConstPtrT> { typedef ConstPtrT ElementType; };
            typedef std::vector<ConstPtrT>                     ConstPtrsT;
            typedef std::vector<PtrT>                          PtrsT;
            typedef std::vector<ConstPtrsT>                    ListOfConstPtrsT;

            inline const CloudT& getPoints () const { return _points;  }
            inline       CloudT& getPoints ()       { return _points;  }
            inline const CloudT& getColors () const { return _colors;  }
            inline       CloudT& getColors ()       { return _colors;  }
            inline bool          hasColors() const { return _colors.size(); }
            inline const CloudT& getNormals() const { return _normals; }
            inline       CloudT& getNormals()       { return _normals; }
            inline bool          hasNormals() const { return _normals.size(); }

            ColoredPointCloud()                                 = default;
            ColoredPointCloud( const ColoredPointCloud& other ) = default;
            inline ColoredPointCloud( const CloudT& points, const CloudT& colors ) : _points(points), _colors(colors) {}
            inline ColoredPointCloud( const size_t   rows, const size_t   cols = CloudT::Dim ) : _points( CloudT::Zero(rows, cols) ), _colors( CloudT::Zero(rows,cols) ), _normals( CloudT::Zero(rows,cols) ) {}
            //template <typename _MeshT>
            //ColoredPointCloud( const _MeshT& mesh ) { typedef typename _MeshT::VecT VecT; CloudT::fromMesh( mesh, this->getPoints(), this->getColors(), this->getNormals() ); }
//            template <typename _MeshT>
//            static inline PtrT fromMesh( const _MeshT& mesh ) { Ptr cloud( new ColoredPointCloud() ); return fromMesh(mesh,cloud->getPoints(),cloud->getColors(),cloud->getNormals()); }

            inline size_t      getNPoints()           const { return _points .getNPoints(); }
            inline ConstPointT getPoint  ( size_t i ) const { return _points .getPoint(i);  }
            inline PointT      getPoint  ( size_t i )       { return _points .getPoint(i);  }
            inline ConstPointT getColor  ( size_t i ) const { return _colors .getPoint(i);  }
            inline PointT      getColor  ( size_t i )       { return _colors .getPoint(i);  }
            inline ConstPointT getNormal ( size_t i ) const { return _normals.getPoint(i);  }
            inline PointT      getNormal ( size_t i )       { return _normals.getPoint(i);  }

            inline void conservativeResize( const Eigen::Index rows ) {
                this->getPoints ().conservativeResize(rows, Eigen::NoChange );
                if (this->hasColors())
                    this->getColors ().conservativeResize(rows, Eigen::NoChange );
                if (this->hasNormals())
                    this->getNormals().conservativeResize(rows, Eigen::NoChange );
            }

            inline void clearNormals() {
                this->getNormals().resize(0, Eigen::NoChange);
            }

            inline void clearColors() {
                this->getColors().resize(0, Eigen::NoChange);
            }

            template <typename _MeshT>
            static void fromMesh( const _MeshT& mesh, PtrT& cloud )
            {
                cloud.reset( new ColoredPointCloud() );
                CloudT::fromMesh( mesh, cloud->getPoints(), cloud->getColors(), cloud->getNormals() );
            }

            template <typename _MeshT>
            static PtrT fromMesh( const _MeshT& mesh )
            {
                PtrT cloud( new ColoredPointCloud() );
                CloudT::fromMesh( mesh, cloud->getPoints(), cloud->getColors(), cloud->getNormals() );
                return cloud;
            }

            template <typename _MeshT>
            static void fromMesh( const _MeshT& mesh, typename CloudT::Ptr& cloud, typename CloudT::Ptr& color, typename CloudT::Ptr& normals );

            static void cat( PtrT& out, const ConstPtrT& a, const ConstPtrT& b );
            void append( const ConstPtrT& a );

            template <typename _MeshT>
            static void writeMesh( const ConstPtrT& cloud, const std::string& name );

            template <typename _TransformT>
            void transformCloud( const _TransformT& transform, ColoredPointCloud<_Scalar>& outPoints ) const;

            template <typename _MeshT>
            static PtrT read( const std::string& path );

            RowVector3 getCentroid() const;

        protected:
            PointCloud<_Scalar> _points;
            PointCloud<_Scalar> _colors;
            PointCloud<_Scalar> _normals;
        public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
    };
} //...ns Soup

//#include "soup/structures/impl/pointCloudEigen.hpp"

#endif // SOUP_POINTCLOUDEIGEN_H

