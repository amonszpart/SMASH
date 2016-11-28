//
// Created by bontius on 17/02/16.
//

#include "ceres/functorInfo.h"
#include "ceres/cost_function.h"
#include <iostream>

namespace ceres
{
  /**
   * \param[in] name_arg    Name of this functor instance.
   * \param[in] fn          Ceres cost functor.
   * \param[in] data        Pointer to unknowns.
   */
  FunctorInfo::FunctorInfo( const std::string& name_arg, ceres::CostFunction* const fn, std::vector<double*> data )
      : functor( fn ), dataPointers( data ), name( name_arg ) {}

  std::vector<ceres::CeresScalar> FunctorInfo::getResiduals() const
  {
      std::vector<ceres::CeresScalar> residuals( functor->num_residuals(), ceres::CeresScalar(0.) );
      functor->Evaluate(dataPointers.data(), residuals.data(), NULL);
      return residuals;
  } //...getResidualBlocks()
} //...ns ceres


