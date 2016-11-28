//
// Created by bontius on 01/04/16.
//

#include "tracking/phys/physFunctorInfo.h"
#include "tracking/phys/weights.h"
#include "tracking/common/util/energyPlotter.h"
#include "tracking/common/typedefs.h"
#include "ceres/solver.h"
#include "ceres/problem.h"
#include <iostream>
#include <map>
#include <tracking/phys/energyTerms/impl/pointTerm.hpp>
#include "tracking/phys/physIndexer.h"

namespace tracking {
  //get the median of an unordered set of numbers of arbitrary type without modifying the
  //underlying dataset
  template <typename InputIterator>
  typename std::iterator_traits<InputIterator>::value_type calcMedian(InputIterator const cbegin,InputIterator const cend)
  {
      typedef typename std::iterator_traits<InputIterator>::value_type T;

      std::vector<T> data(cbegin, cend);

      // find the median
      std::nth_element(data.begin(), data.begin() + data.size() / 2, data.end(),
          [](T const a, T const b)
          {
              return b > a;
          });

      return data[data.size() / 2];
  }

  namespace bundle_physics {

    std::set<TrackId> solve(ceres::Problem &problem, PhysIndexer const& /*indexer*/, Weights const& weights, PhysFunctorInfosT &/*costFunctors*/) {
        using ceres::CeresScalar;
        ceres::Solver::Options options;
        {
            options.linear_solver_type           = ceres::SPARSE_NORMAL_CHOLESKY; // nonlin lsq
            options.preconditioner_type          = ceres::JACOBI;
            //options.linear_solver_type           = ceres::SPARSE_SCHUR;
            //options.preconditioner_type          = ceres::CLUSTER_JACOBI;
            //options.dynamic_sparsity = true;
            options.minimizer_progress_to_stdout = true;
            options.num_threads                  = 16;

            options.max_num_iterations           = weights.max_num_iterations;
            //options.max_solver_time_in_seconds   = 30;
            options.function_tolerance           = weights.function_tolerance; // 16
            options.parameter_tolerance          = weights.parameter_tolerance; // 16
            options.gradient_tolerance           = options.function_tolerance * 1.e-4;
            options.max_solver_time_in_seconds   = weights.max_solver_time_in_seconds;
            //std::cout << "linsearch: " << options.line_search_direction_type << std::endl;
            //options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
            //std::cout << "linsearch: " << options.line_search_direction_type << std::endl;
        }

        std::set<TrackId> inliers;
        ceres::Solver::Summary summary;
        if (!(weights.solveFlags & Weights::NO_SOLVE)) {
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
        }

#if 0
        if (weights.solveFlags ^ Weights::NO_SOLVE) {
            std::cout << "[" << __func__ << "] selecting 2dD functors..." << std::endl;
            std::map<TrackId, std::vector<std::shared_ptr<PhysFunctorInfo> > > functors;
            for ( auto it = costFunctors.begin(); it != costFunctors.end(); ++it) {
                std::shared_ptr<PhysFunctorInfo> functorInfo = (*it);
                std::shared_ptr<TrackPhysFunctorInfo> trackInfo = std::dynamic_pointer_cast<TrackPhysFunctorInfo>(functorInfo);
                if (trackInfo) {
                    functors[trackInfo->getTrackId()].push_back(trackInfo);
                    if (weights.poseIntegralSteps < 1.) {
                        ceres::CostFunction const *costFunction = problem.GetCostFunctionForResidualBlock(trackInfo->getResidualBlockId());
                        PointCostFunctor::AutoDiffType const *cAutoDiffFunctor = reinterpret_cast<PointCostFunctor::AutoDiffType const *>(costFunction);
                        PointCostFunctor::AutoDiffType *autoDiffFunctor = const_cast<PointCostFunctor::AutoDiffType*>(cAutoDiffFunctor);

                        if (autoDiffFunctor->getCostFunctor()->getIntegrationSteps() < 1)
                            autoDiffFunctor->getCostFunctor()->setIntegrationSteps(std::max(1,static_cast<int>(std::round(
                                autoDiffFunctor->getCostFunctor()->getIntegrationSteps() * 1.2 ))));
                            //autoDiffFunctor->getCostFunctor()->setIntegrationSteps(std::max(1,static_cast<int>(std::round(autoDiffFunctor->getCostFunctor()->getIntegrationSteps() / weights.poseIntegralSteps))));

                        //problem.SetParameterBlockConstant(trackInfo->dataPointers.at(0));
                    }
                }
            }
            //std::cout << "[" << __func__ << "] fixed points!" << std::endl;

            double cost(0.);
            std::multiset<double> costs;
            //EnergyPlotter costsPlotter;
            std::map<TrackId, double> trackIdToCost;
            std::vector<double> residuals;
            for (auto const& trackIdAndFunctorList : functors) {
                TrackId const& trackId = trackIdAndFunctorList.first;
                std::vector<std::shared_ptr<PhysFunctorInfo> > const& functorList = trackIdAndFunctorList.second;

                std::vector<ceres::ResidualBlockId> residualBlockIds(functorList.size());
                std::transform(functorList.begin(),functorList.end(),residualBlockIds.begin(),[](std::shared_ptr<PhysFunctorInfo> const& pfi) {
                    return pfi->getResidualBlockId();
                });
                ceres::Problem::EvaluateOptions evaluateOptions;
                evaluateOptions.residual_blocks = residualBlockIds;
                problem.Evaluate(evaluateOptions,&cost, &residuals, nullptr,nullptr);
                //std::cout << "residuals.size():" << residuals.size() << " vs. numFunctors" << residualBlockIds.size() << "(" << functorList.size() << ")" << std::endl;
                cost /= static_cast<CeresScalar>(residualBlockIds.size());
                //costsPlotter.addValue(cost,"TrackResidualUnfiltered");
                costs.insert(cost);
                trackIdToCost[trackId] = cost;
            }
            //costsPlotter.plot();
#if 1
            for (auto const& pair : trackIdToCost) {
                TrackId const &trackId = pair.first;
                inliers.insert(trackId);
            }
#else
            std::cout << "[" << __func__ << "] statring median calculation..." << std::endl;
            double const median = calcMedian(costs.begin(),costs.end());
            std::cout << "[" << __func__ << "] median cost: " << median << std::endl;
            double const threshold = 2.0 * median;

            EnergyPlotter filteredCostsPlotter;
            for (auto const& pair : trackIdToCost){
                TrackId const& trackId = pair.first;
                double cost = pair.second;
                if (cost > threshold){
                    for (auto const& pfi : functors.at(trackId)){
                        problem.RemoveResidualBlock(pfi->getResidualBlockId());
                        //TODO: remove from costFunctors
                    }
                } else {
                    inliers.insert(trackId);
                    filteredCostsPlotter.addValue(cost,"TrackResidualFiltered");
                }
            }
            filteredCostsPlotter.plot();

            std::cout << "releasing mass" << std::endl;
            problem.SetParameterBlockVariable( const_cast<CeresScalar*>(indexer.getMassConst(1)) );
#endif
//            std::cout << "releasing collision point" << std::endl;
//            problem.SetParameterBlockVariable( const_cast<CeresScalar*>(indexer.getCollisionPointConst(0)) );
            ceres::Solve(options, &problem, &summary);
            std::cout << summary.FullReport() << "\n";
        }
#endif
        return inliers;
    } //...solve()
  } //...ns bundle_phsyics
} //...ns tracking