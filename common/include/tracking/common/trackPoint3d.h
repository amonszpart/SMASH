//
// Created by bontius on 25/05/16.
//

#ifndef TRACKVIDEO_COMMON_TRACKPOINT3D_H
#define TRACKVIDEO_COMMON_TRACKPOINT3D_H

#include "tracking/common/eigen.h"
#include "tracking/common/trackPointFwDecl.h"

namespace tracking {

  template <typename _Scalar>
  class TrackPoint3DTemplate {
          enum { NeedsToAlign = (sizeof(_Scalar)%16)==0 };
      public:
          enum { RowsAtCompileTime = 9, ColsAtCompileTime = 1 };
          typedef _Scalar Scalar;
          //typedef Eigen::Matrix< Scalar, 6, 1 > Base;
          typedef Eigen::Matrix< Scalar, 3, 1 > LocationT;
          typedef Eigen::Matrix< Scalar, 3, 1 > NormalT;
          typedef Eigen::Matrix< float , 3, 1 > ColorT;
#if 0
          TrackPoint3D(void) : Base(), _hasColor(false) {}

          // This constructor allows you to construct MyVectorType from Eigen expressions
          template<typename OtherDerived>
          TrackPoint3D(Eigen::MatrixBase<OtherDerived> const& other)
              : Base(other), _hasColor(false)
          {}
          // This method allows you to assign Eigen expressions to MyVectorType
          template<typename OtherDerived>
          TrackPoint3D& operator= (const Eigen::MatrixBase <OtherDerived>& other) {
              this->Base::operator=(other);
              this->_hasColor = false;
              return *this;
          }
          using Base::Base;
#endif
          TrackPoint3DTemplate();

          template <typename _DerivedPointT>
          explicit TrackPoint3DTemplate(Eigen::MatrixBase<_DerivedPointT> const& point);

          template <typename _DerivedPointT, typename _DerivedNormalT>
          explicit TrackPoint3DTemplate(Eigen::MatrixBase<_DerivedPointT> const& point, Eigen::MatrixBase<_DerivedNormalT> const& normal);

          template <typename _DerivedPointT, typename _DerivedNormalT, typename _DerivedColorT>
          explicit TrackPoint3DTemplate(Eigen::MatrixBase<_DerivedPointT> const& point,
                                        Eigen::MatrixBase<_DerivedNormalT> const& normal,
                                        Eigen::MatrixBase<_DerivedColorT> const& color);

          TrackPoint3DTemplate(TrackPoint3DTemplate&& other) = default;
          TrackPoint3DTemplate(TrackPoint3DTemplate const& other) = default;
          TrackPoint3DTemplate& operator=(TrackPoint3DTemplate const& other) = default;
          TrackPoint3DTemplate& operator=(TrackPoint3DTemplate && other) = default;

          template <typename _DerivedT>
          static TrackPoint3DTemplate WithNoNormal( const Eigen::MatrixBase<_DerivedT>& pnt );

          template <typename _DerivedAT, typename _DerivedBT>
          static TrackPoint3DTemplate WithNormal(const Eigen::MatrixBase<_DerivedAT>& pnt, const Eigen::MatrixBase<_DerivedBT>& normal);

          template <typename _DerivedAT, typename _DerivedBT, typename _DerivedColorT>
          static TrackPoint3DTemplate WithAll(const Eigen::MatrixBase<_DerivedAT>& pnt, Eigen::MatrixBase<_DerivedBT> const& normal, Eigen::MatrixBase<_DerivedColorT> const& color);

          LocationT const& getPoint() const;
          LocationT const& getPos() const;
          LocationT      & getPoint();
          template <typename _DerivedT>
          void setPoint(Eigen::MatrixBase<_DerivedT> const& point);
          template <typename _DerivedT>
          void setPos(Eigen::MatrixBase<_DerivedT> const& point);

          NormalT const& getNormal() const;
          template <typename _DerivedT>
          void setNormal(Eigen::MatrixBase<_DerivedT> const& normal);
          bool hasNormal() const;

          ColorT const& getColor() const;
          template <typename _DerivedT>
          void setColor(Eigen::MatrixBase<_DerivedT> const& color);
          bool hasColor() const;

          Scalar const& operator()(int id) const;
          Scalar      & operator()(int id);

      protected:
          LocationT _point;
          NormalT   _normal;
          ColorT    _color;
          bool      _hasNormal;
          bool      _hasColor;
      public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  }; //...TrackPoint3DTemplate

} //...namespace tracking

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(tracking::TrackPoint3DTemplate<float>)
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(tracking::TrackPoint3DTemplate<double>)

#endif //TRACKVIDEO_COMMON_TRACKPOINT3D_H
