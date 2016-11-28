#include "tracking/common/correspondence.h"

namespace tracking {
  bool
  tracking::Correspondence::computeTransform(TransformationT& transform, Tracks3D const& tracks,
                                             const CorrPairs& corrs) {
      Eigen::MatrixXd P(3, corrs.size());
      Eigen::MatrixXd Q(3, corrs.size());

      // get centroid
      Eigen::Vector3d centroid0(Eigen::Vector3d::Zero());
      Eigen::Vector3d centroid1(Eigen::Vector3d::Zero());
      for (CorrPair const& corrPair : corrs) {
          centroid0 += tracks.getTrack(corrPair.getSource().getTrackId()).getPoint(
              corrPair.getSource().getFrameId()).getPoint().template cast<double>();
          centroid1 += tracks.getTrack(corrPair.getTarget().getTrackId()).getPoint(
              corrPair.getTarget().getFrameId()).getPoint().template cast<double>();
      }
      centroid0 /= static_cast<double>( corrs.size());
      centroid1 /= static_cast<double>( corrs.size());

      Eigen::Matrix3d H(Eigen::Matrix3d::Zero());
      for (CorrPair const& corrPair : corrs) {
          H += (tracks.getTrack(corrPair.getSource().getTrackId()).getPoint(
              corrPair.getSource().getFrameId()).getPoint().template cast<double>() - centroid0) *
               (tracks.getTrack(corrPair.getTarget().getTrackId()).getPoint(
                   corrPair.getTarget().getFrameId()).getPoint().template cast<double>() - centroid1).transpose();
      }

      // Assemble the correlation matrix H = source * target'
      // Compute the Singular Value Decomposition
      Eigen::JacobiSVD<Eigen::Matrix3d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
      Eigen::Matrix3d                   u = svd.matrixU();
      Eigen::Matrix3d                   v = svd.matrixV();

      // Compute R = V * U'
      if (u.determinant() * v.determinant() < 0) {
          for (int x = 0; x < 3; ++x) {
              v(x, 2) *= -1.;
          }
      }

      Eigen::Matrix3d R = v * u.transpose();

      //TransformationType transformation_matrix;
      transform.setIdentity();
      // Return the correct transformation
      transform.matrix().topLeftCorner<3, 3>() = R.cast<float>();
      transform.matrix().block<3, 1>(0, 3)     = (centroid1 - R * centroid0).cast<float>();

      //return transformation_matrix;
      return true;
  } //...computeTransform()
} //...ns tracking

