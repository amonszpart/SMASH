//
// Created by bontius on 25/05/16.
//

#include "tracking/common/impl/trackPoint3d.hpp"

namespace tracking {
using Soup::geometry::Scalar;

template
class TrackPoint3DTemplate<Scalar>;

template TrackPoint3DTemplate<Scalar>::TrackPoint3DTemplate(
    Eigen::MatrixBase<TrackPoint3D::LocationT> const&);

template TrackPoint3DTemplate<Scalar> TrackPoint3DTemplate<Scalar>::WithNoNormal(
    Eigen::MatrixBase<Eigen::CwiseUnaryOp<Eigen::internal::scalar_cast_op<double, Scalar>,
        Eigen::Vector3d const> > const&);

template TrackPoint3DTemplate<Scalar> TrackPoint3DTemplate<Scalar>::WithNoNormal(
    Eigen::MatrixBase<TrackPoint3D::LocationT> const&);

template TrackPoint3DTemplate<Scalar> TrackPoint3DTemplate<Scalar>::WithNoNormal(
    Eigen::MatrixBase<Eigen::CwiseUnaryOp<
        Eigen::internal::scalar_cast_op<double, Scalar>,
        Eigen::Map<Eigen::Matrix<double, 3, 1> const, 0, Eigen::Stride<0, 0> > const> > const&);

template TrackPoint3DTemplate<Scalar> TrackPoint3DTemplate<Scalar>::WithNoNormal(
    Eigen::MatrixBase<Eigen::CwiseUnaryOp<
        Eigen::internal::scalar_cast_op<double, Scalar>,
        Eigen::Map<Eigen::Matrix<double, 3, 1>, 0, Eigen::Stride<0, 0> > const> > const&);

template TrackPoint3DTemplate<Scalar>::TrackPoint3DTemplate(
    Eigen::MatrixBase<TrackPoint3D::LocationT> const&,
    Eigen::MatrixBase<TrackPoint3D::NormalT> const&,
    Eigen::MatrixBase<TrackPoint3D::ColorT> const&);

template TrackPoint3DTemplate<Scalar>::TrackPoint3DTemplate(
    Eigen::MatrixBase<TrackPoint3D::LocationT> const&,
    Eigen::MatrixBase<Eigen::Map<const Eigen::Vector3f> > const&,
    Eigen::MatrixBase<TrackPoint3D::ColorT> const&);

template TrackPoint3DTemplate<Scalar>::TrackPoint3DTemplate(
    Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 1> const, 1, 3, true> > const&,
    Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 1> const, 1, 3, true> > const&,
    Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 1> const, 1, 3, true> > const&);

template TrackPoint3DTemplate<Scalar>::TrackPoint3DTemplate(
    Eigen::MatrixBase<Eigen::CwiseUnaryOp<Eigen::internal::scalar_cast_op<double, Scalar>, Eigen::Vector3d const> > const&,
    Eigen::MatrixBase<Eigen::Matrix<Scalar, 3, 1> > const&,
    Eigen::MatrixBase<Eigen::Matrix<Scalar, 3, 1> > const&);

template TrackPoint3DTemplate<Scalar>::TrackPoint3DTemplate(
    Eigen::MatrixBase<Eigen::Matrix<Scalar, 3, 1> > const&,
    Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 1, Eigen::Dynamic, 3> const, 1, 3, true> > const&,
    Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, Eigen::Dynamic, 3, 1, Eigen::Dynamic, 3> const, 1, 3, true> > const&);

template TrackPoint3DTemplate<Scalar>::TrackPoint3DTemplate(
    Eigen::MatrixBase<Eigen::CwiseUnaryOp<Eigen::internal::scalar_cast_op<double, Scalar>, Eigen::Vector3d const> > const&,
    Eigen::MatrixBase<Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<Scalar>, Eigen::Matrix<Scalar, 3, 1> > > const&,
    Eigen::MatrixBase<Eigen::CwiseNullaryOp<Eigen::internal::scalar_constant_op<Scalar>, Eigen::Matrix<Scalar, 3, 1> > > const&);

template void TrackPoint3DTemplate<Scalar>::setPoint(Eigen::MatrixBase<Eigen::Matrix<Scalar, 3, 1> > const&);
template void TrackPoint3DTemplate<Scalar>::setPos(Eigen::MatrixBase<Eigen::Matrix<Scalar, 3, 1> > const&);
template void TrackPoint3DTemplate<Scalar>::setPos(Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, -1, 3, 1, -1, 3> const, 1, 3, true> > const&);
template void TrackPoint3DTemplate<Scalar>::setNormal(Eigen::MatrixBase<Eigen::Matrix<Scalar, 3, 1> > const&);
template void TrackPoint3DTemplate<Scalar>::setNormal(Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, -1, 3, 1, -1, 3> const, 1, 3, true> > const&);
template void TrackPoint3DTemplate<Scalar>::setColor(Eigen::MatrixBase<Eigen::Block<Eigen::Matrix<Scalar, -1, 3, 1, -1, 3> const, 1, 3, true> > const&);
} //...ns tracking

