//
// Created by bontius on 25/05/16.
//

#ifndef TRACKVIDEO_TRACKPOINT3D_HPP
#define TRACKVIDEO_TRACKPOINT3D_HPP

#include "tracking/common/trackPoint3d.h"
#include <iostream>

namespace tracking {

template <typename _Scalar>
TrackPoint3DTemplate<_Scalar>::TrackPoint3DTemplate()
  : _point(LocationT::Zero()), _hasNormal(false), _hasColor(false)
{}

template <typename _Scalar>
template <typename _DerivedPointT>
TrackPoint3DTemplate<_Scalar>::TrackPoint3DTemplate(Eigen::MatrixBase<_DerivedPointT> const& point)
  : _point(point.derived()), _hasNormal(false), _hasColor(false)
{ static_assert( _DerivedPointT::RowsAtCompileTime + _DerivedPointT::ColsAtCompileTime == 4, "Need 3 coordinates for point"); }

template <typename _Scalar>
template <typename _DerivedPointT, typename _DerivedNormalT>
TrackPoint3DTemplate<_Scalar>::TrackPoint3DTemplate(Eigen::MatrixBase<_DerivedPointT> const& point, Eigen::MatrixBase<_DerivedNormalT> const& normal)
  : _point(point.derived()), _normal(normal.derived()), _hasNormal(true), _hasColor(false)
{   static_assert( _DerivedPointT::RowsAtCompileTime + _DerivedPointT::ColsAtCompileTime == 4, "Need 3 coordinates for point");
  static_assert( _DerivedNormalT::RowsAtCompileTime + _DerivedNormalT::ColsAtCompileTime == 4, "Need 3 coordinates for normal"); }

template <typename _Scalar>
template <typename _DerivedPointT, typename _DerivedNormalT, typename _DerivedColorT>
TrackPoint3DTemplate<_Scalar>::TrackPoint3DTemplate(Eigen::MatrixBase<_DerivedPointT> const& point, Eigen::MatrixBase<_DerivedNormalT> const& normal, Eigen::MatrixBase<_DerivedColorT> const& color)
  : _point(point.derived()), _normal(normal.derived()), _color(color.derived()), _hasNormal(true), _hasColor(true)
{   static_assert( _DerivedPointT::RowsAtCompileTime + _DerivedPointT::ColsAtCompileTime == 4, "Need 3 coordinates for point");
  static_assert( _DerivedNormalT::RowsAtCompileTime + _DerivedNormalT::ColsAtCompileTime == 4, "Need 3 coordinates for normal");
  static_assert( _DerivedColorT::RowsAtCompileTime + _DerivedColorT::ColsAtCompileTime == 4, "Need 3 coordinates for color"); }

template <typename _Scalar>
template <typename _DerivedT>
TrackPoint3DTemplate<_Scalar> TrackPoint3DTemplate<_Scalar>::WithNoNormal(Eigen::MatrixBase<_DerivedT> const& pnt) {
  static_assert( _DerivedT::RowsAtCompileTime + _DerivedT::ColsAtCompileTime == 4, "Need 3 coordinates" );
  return TrackPoint3DTemplate<_Scalar>{pnt};
} //...WithNoNormal()

template <typename _Scalar>
template <typename _DerivedAT, typename _DerivedBT>
TrackPoint3DTemplate<_Scalar> TrackPoint3DTemplate<_Scalar>::WithNormal(Eigen::MatrixBase<_DerivedAT> const& pnt, Eigen::MatrixBase<_DerivedBT> const& normal) {
  static_assert( _DerivedAT::RowsAtCompileTime + _DerivedAT::ColsAtCompileTime == 4, "Need 3 coordinates for point" );
  static_assert( _DerivedBT::RowsAtCompileTime + _DerivedBT::ColsAtCompileTime == 4, "Need 3 coordinates for normal" );
  return TrackPoint3D(pnt, normal);
} //...WithNormal()

template <typename _Scalar>
template <typename _DerivedAT, typename _DerivedBT, typename _DerivedColorT>
TrackPoint3DTemplate<_Scalar> TrackPoint3DTemplate<_Scalar>::WithAll(Eigen::MatrixBase<_DerivedAT> const& pnt, Eigen::MatrixBase<_DerivedBT> const& normal, Eigen::MatrixBase<_DerivedColorT> const& color) {
  static_assert( _DerivedAT::RowsAtCompileTime + _DerivedAT::ColsAtCompileTime == 4, "Need 3 coordinates for point" );
  static_assert( _DerivedBT::RowsAtCompileTime + _DerivedBT::ColsAtCompileTime == 4, "Need 3 coordinates for normal" );
  static_assert( _DerivedColorT::RowsAtCompileTime + _DerivedColorT::ColsAtCompileTime == 4, "Need 3 coordinates for color" );
  TrackPoint3DTemplate point3d;
  point3d << pnt,normal;
  point3d.setColor(color);
  return point3d;
} //...WithAll()

template <typename _Scalar>
inline typename TrackPoint3DTemplate<_Scalar>::LocationT const& TrackPoint3DTemplate<_Scalar>::getPoint() const
{ return _point; } //...getPoint()

template <typename _Scalar>
inline typename TrackPoint3DTemplate<_Scalar>::LocationT      & TrackPoint3DTemplate<_Scalar>::getPoint()
{ return _point; } //...getPoint()

template <typename _Scalar>
inline typename TrackPoint3DTemplate<_Scalar>::LocationT const& TrackPoint3DTemplate<_Scalar>::getPos() const
{ return _point; } //...getPos()

template <typename _Scalar>
template <typename _DerivedT>
inline void TrackPoint3DTemplate<_Scalar>::setPoint(Eigen::MatrixBase<_DerivedT> const& point)
{ _point = point.derived(); } //...setPoint()

template <typename _Scalar>
template <typename _DerivedT>
inline void TrackPoint3DTemplate<_Scalar>::setPos(Eigen::MatrixBase<_DerivedT> const& point)
{ _point = point.derived(); } //...setPoint()

template <typename _Scalar>
inline typename TrackPoint3DTemplate<_Scalar>::NormalT const& TrackPoint3DTemplate<_Scalar>::getNormal() const {
  if (!_hasNormal)
      std::cerr << "[" << __func__ << "] " << "no normal set" << std::endl;
  return _normal;
} //...getNormal()

template <typename _Scalar>
template <typename _DerivedT>
inline void TrackPoint3DTemplate<_Scalar>::setNormal(Eigen::MatrixBase<_DerivedT> const& normal)
{ _normal = normal.derived(); _hasNormal = true; } //...setNormal()

template <typename _Scalar>
inline bool TrackPoint3DTemplate<_Scalar>::hasNormal() const
{ return _hasNormal; } //...hasNormal()

template <typename _Scalar>
inline typename TrackPoint3DTemplate<_Scalar>::ColorT const& TrackPoint3DTemplate<_Scalar>::getColor() const {
  if (!_hasColor)
      std::cerr << "[" << __func__ << "] " << "no color set" << std::endl;
  return _color;
} //...getColor()

template <typename _Scalar>
template <typename _DerivedT>
inline void TrackPoint3DTemplate<_Scalar>::setColor(Eigen::MatrixBase<_DerivedT> const& color) {
  _color = color.derived();
  _hasColor = true;
} //...setColor()

template <typename _Scalar>
inline bool TrackPoint3DTemplate<_Scalar>::hasColor() const
{ return _hasColor; } //...hasColor()

template <typename _Scalar>
inline typename TrackPoint3DTemplate<_Scalar>::Scalar const& TrackPoint3DTemplate<_Scalar>::operator()(int id) const {
  if (id < 3)
      return _point(id);
  else if (id < 6) {
      if (!_hasNormal)
          std::cerr << "[" << __func__ << "] " << "no normal set" << std::endl;
      return _normal(id - 3);
  } else {
      if (!_hasColor)
          std::cerr << "[" << __func__ << "] " << "no color set" << std::endl;
      return _color(id - 6);
  }
} //...operator()

template <typename _Scalar>
inline typename TrackPoint3DTemplate<_Scalar>::Scalar& TrackPoint3DTemplate<_Scalar>::operator()(int id) {
  return const_cast<Scalar&>(static_cast<TrackPoint3D&>(*this).operator()(id));
} //...operator()

} //...namespace tracking

#endif //TRACKVIDEO_TRACKPOINT3D_HPP
