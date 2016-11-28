//
// Created by bontius on 11/01/16.
//
#include "tracking/annot/cuboid.h"
#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"

namespace tracking {
namespace bundle_physics {

void addGravityTerm( ceres::Problem& problem, PhysIndexer& indexer, ceres::FunctorInfosT& costFunctors, const Weights& weights, const Consts& consts ) {
    char name[512];

    ceres::CeresScalar *parabolaA = indexer.getParabolaSquaredParam();
    // init
    // g = 2. * a * fps * fps = 9.81
    // a = g / (2.*fps*fps)
    parabolaA[0] = 9.81 / (2. * consts.k_fps * consts.k_fps); // y is flipped due to image coordinates increasing downwards
    //if ( weights.fixFlags & Weights::FIX_GRAVITY_AND_A )
    problem.SetParameterBlockConstant( parabolaA );

//    {
//        ceres::CostFunction *costFunction = GravityCostFunctor::Create(weights.gravityDownWeight, consts.k_fps, 9.81);
//        problem.AddResidualBlock(costFunction, NULL, parabolaA);
//
//        sprintf(name, "GravityError");
//        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {parabolaSharedParams}));
//    }

    ceres::CeresScalar *parabolaRotShared = indexer.getParabolaRotationShared();
    {
        ceres::CostFunction *costFunction = GravityDownCostFunctor::Create(weights.gravityDownWeight, consts.k_fps, Vector3(0.,1.,0.) );
        problem.AddResidualBlock(costFunction, NULL, parabolaRotShared, parabolaA );

        sprintf(name, "GravityDownError");
        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {parabolaRotShared, parabolaA}));

    }
} //...addGravityTerm()

void initGravityTerm(ceres::Problem &/*problem*/, PhysIndexer &indexer, double const k_fps) {
    // g = 2. * a * fps * fps = 9.81
    // a = g / (2.*fps*fps)
    ceres::CeresScalar *parabolaA = indexer.getParabolaSquaredParam();
    parabolaA[0] = 9.81 / (2. * k_fps * k_fps); // y is flipped due to image coordinates increasing downwards
    //if ( weights.fixFlags & Weights::FIX_GRAVITY_AND_A )
    //problem.SetParameterBlockConstant(parabolaA);
}

} //...ns bundle_physics
} //...ns tracking