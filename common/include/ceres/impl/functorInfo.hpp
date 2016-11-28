//
// Created by bontius on 22/02/16.
//

#ifndef TRACKVIDEO_FUNCTORINFO_HPP
#define TRACKVIDEO_FUNCTORINFO_HPP

#include "ceres/typedefs.h"
#include "ceres/cost_function.h"
#include <iostream>

namespace ceres {
  template <typename _FunctorInfosT>
  void printFunctors( _FunctorInfosT const& costFunctors )
  {
      std::vector<ceres::CeresScalar> residuals;
      for ( auto const &info : costFunctors )
      {
          const ceres::CostFunction *fn = info.functor;
          std::cout << info.name;
          if ( int(residuals.size()) < fn->num_residuals() )
              residuals.resize( fn->num_residuals() );
          fn->Evaluate(info.dataPointers.data(), residuals.data(), NULL);
          for (int ri = 0; ri != fn->num_residuals(); ++ri)
              std::cout << " " << residuals[ri];
          std::cout << std::endl;
      } //...for costFunctors
  } //...printFunctors()
} //...ns tracking

#endif //TRACKVIDEO_FUNCTORINFO_HPP
