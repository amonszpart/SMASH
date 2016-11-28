//
// Created by bontius on 02/02/16.
//

#ifndef TRACKVIDEO_FUNCTORINFO_H
#define TRACKVIDEO_FUNCTORINFO_H

#include "ceres/typedefs.h"
#include <vector>
#include <string>
#include <iostream>

namespace ceres {
  class CostFunction;

  /** \brief Enables residual energy estimation after optimisation. */
  struct FunctorInfo {
      /** \brief FunctorInfo constructor. */
      FunctorInfo( const std::string& name_arg, ceres::CostFunction* const fn, std::vector<double*> data );
      virtual ~FunctorInfo() = default;
      std::vector<ceres::CeresScalar> getResiduals() const;

      ceres::CostFunction                  *functor; //!< Pointer to cost functor.
      std::vector<ceres::CeresScalar*> dataPointers; //!< List of pointers to unknowns.
      std::string                              name; //!< Unique functor identifier name. (Used as key in a map).
  };
  typedef std::vector<FunctorInfo> FunctorInfosT;
} //...ns ceres

#endif //TRACKVIDEO_FUNCTORINFO_H
