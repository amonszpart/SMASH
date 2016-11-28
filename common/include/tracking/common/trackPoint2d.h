//
// Created by bontius on 29/05/16.
//

#ifndef TRACKVIDEO_COMMON_TRACKPOINT2D_H
#define TRACKVIDEO_COMMON_TRACKPOINT2D_H

#include "tracking/common/eigen.h"
#include "tracking/common/typedefs.h"

namespace tracking {
  /** \brief Screen space point class. */
  class TrackPoint2D : public Eigen::Vector2f {
      public:
          typedef Eigen::Vector2f Base;
          typedef Eigen::Vector2f MyBase;
          typedef TrackPoint2D    Point2T;
          typedef Base            LocationT;
          enum { Dim = 2 };

          static size_t constexpr SizeInBinary() {
              return sizeof(Base::Scalar) * Base::RowsAtCompileTime + sizeof(FrameId);
          }

          TrackPoint2D(void)
              : Base() {}

          // This constructor allows you to construct MyVectorType from Eigen expressions
          template<typename OtherDerived>
          TrackPoint2D(const Eigen::MatrixBase<OtherDerived>& other)
              : Base(other) {}

          // This method allows you to assign Eigen expressions to MyVectorType
          template<typename OtherDerived>
          TrackPoint2D& operator=(const Eigen::MatrixBase<OtherDerived>& other) {
              this->Base::operator=(other);
              return *this;
          }

          using Base::Base;

          Base const& getPoint() const { return *this; }
          Base const& getPos() const { return *this; }

          Base& getPoint() { return *this; }

      public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  }; //...TrackPoint2D
} //...ns tracking
EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(tracking::TrackPoint2D)

#endif //TRACKVIDEO_COMMON_TRACKPOINT2D_H
