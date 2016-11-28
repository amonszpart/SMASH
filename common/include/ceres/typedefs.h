//
// Created by bontius on 02/02/16.
//

#ifndef TRACKVIDEO_CERES_TYPEDEFS_H
#define TRACKVIDEO_CERES_TYPEDEFS_H

#include "tracking/common/eigen.h"

namespace ceres {
  /** \addtogroup common
   * @{ */

  /** \addtogroup Ceres-addons
   * @{ */

  typedef double                           CeresScalar;          //!< Floating point type used by ceres during optimization (double).
  using Scalar = CeresScalar;                                    //!< Alias to \ref CeresScalar.
  typedef Eigen::Matrix< CeresScalar,3,1>  CeresVector3;         //!< 3D vector using \ref CeresScalar.
  using Vector3 = CeresVector3;                                  //!< Alias for \ref CeresVector3.
  typedef Eigen::Matrix< CeresScalar,4,1>  CeresVector4;         //!< 4D vector using \ref CeresScalar.
  typedef Eigen::Map<       CeresVector3 > MapCeresVector3;      //!< Map to 3D vector using \ref CeresScalar.
  typedef Eigen::Map< const CeresVector3 > MapConstCeresVector3; //!< Const map to 3D vector using \ref CeresScalar.
  typedef Eigen::Map<       CeresVector4 > MapCeresVector4;      //!< Map to 4D vector using \ref CeresScalar.
  typedef Eigen::Map< const CeresVector4 > MapConstCeresVector4; //!< Const map to 4D vector using \ref CeresScalar.
  typedef Eigen::Matrix< CeresScalar,3,3>  CeresMatrix3;         //!< 3x3 matrix using \ref CeresScalar.

  /** @} (common) */
  /** @} (Ceres-addons)*/
} //...ns ceres

#endif //TRACKVIDEO_CERES_TYPEDEFS_H
