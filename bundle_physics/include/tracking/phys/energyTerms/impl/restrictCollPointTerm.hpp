//
// Created by bontius on 10/01/16.
//

#ifndef TRACKVIDEO_PHYS_RESTRICTCOLLPOINTTERM_HPP
#define TRACKVIDEO_PHYS_RESTRICTCOLLPOINTTERM_HPP

#include "tracking/phys/energyTerms/restrictCollPointTerm.h"
#include "tracking/annot/cuboid.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/weights.h"

namespace tracking {
  namespace bundle_physics {

    struct CollPointCostFunctor {
            DEFINE_EXCEPTION(CollPointCostFunctor_ParticipantIdLargerThan1)

            enum { NUM_RESIDUALS = 6 };
            typedef ceres::CeresScalar CeresScalar;
        public:
            template <typename _Vector3>
            CollPointCostFunctor( const ceres::CeresScalar sqrtWeight, const _Vector3& sizeA, const _Vector3& sizeB )
                : _sqrtWeight( sqrtWeight )
                  , _sizeA( {sizeA[0], sizeA[1], sizeA[2]} )
                  , _sizeB( {sizeB[0], sizeB[1], sizeB[2]} )
                  , _radiusA( std::max( sizeA[0], std::max(sizeA[1],sizeA[2]))/2. )
                  , _radiusB( std::max( sizeB[0], std::max(sizeB[1],sizeB[2]))/2. )
            {}

            template <typename T>
            bool operator()(
                const T         collPoint[3], // relative collision vector w.r.t. centroid of A (collPointWorld = collPoint + centroidA)
                const T         translationA[3],
                const T         translationB[3],
                /*const T* const  collTime,*/
                T* residuals ) const
            {
#if 0
                T d[3];
                d[0] = translationA[0] + collPoint[0] - (translationA[0] + translationB[0])/T(2.);
                d[1] = translationA[1] + collPoint[1] - (translationA[1] + translationB[1])/T(2.);
                d[2] = translationA[2] + collPoint[2] - (translationA[2] + translationB[2])/T(2.);
                T len = ceres::length(d);
                if ( ceres::abs(len) > T(_radius) )
                {
                    residuals[0] = T(_sqrtWeight) * d[0];
                    residuals[1] = T(_sqrtWeight) * d[1];
                    residuals[2] = T(_sqrtWeight) * d[2];
                }
                else
                {
                    residuals[0] = T(0.);
                    residuals[1] = T(0.);
                    residuals[2] = T(0.);
                }
#else
                T lenA = ceres::length(collPoint);
                if (ceres::abs(lenA) > T(_radiusA))
                {
                    residuals[0] = T(_sqrtWeight) * collPoint[0];
                    residuals[1] = T(_sqrtWeight) * collPoint[1];
                    residuals[2] = T(_sqrtWeight) * collPoint[2];
                }
                else
                {
                    residuals[0] = T(0.); residuals[1] = T(0.); residuals[2] = T(0.);
                }

                T xb[3]; // xb = ca + xa - cb
                xb[0] = translationA[0] + collPoint[0] - translationB[0];
                xb[1] = translationA[1] + collPoint[1] - translationB[1];
                xb[2] = translationA[2] + collPoint[2] - translationB[2];
                T lenB = ceres::length(xb);
                if (ceres::abs(lenB) > T(_radiusB))
                {
                    residuals[3] = T(_sqrtWeight) * xb[0];
                    residuals[4] = T(_sqrtWeight) * xb[1];
                    residuals[5] = T(_sqrtWeight) * xb[2];
                }
                else
                {
                    residuals[3] = T(0.); residuals[4] = T(0.); residuals[5] = T(0.);
                }
#endif

                //std::cout << "len: " << ceres::printJet(len) << ", radius: " << _radius << ", d: " << ceres::printJetVector3(d) << std::endl;

                return true;
            } //...operator()

            template <typename _Vector3>
            static ceres::CostFunction* Create( const ceres::CeresScalar sqrtWeight, const _Vector3& sizeA, const _Vector3& sizeB )
            {
                return new ceres::AutoDiffCostFunction<CollPointCostFunctor,
                    NUM_RESIDUALS,
                    PhysIndexer::COLL_POINTS_STRIDE,
                    //PhysIndexer::PARABOLA_SHARED_STRIDE,
                    PhysIndexer::PARABOLA_TRANSLATION_STRIDE,
                    PhysIndexer::PARABOLA_TRANSLATION_STRIDE
                    /*PhysIndexer::COLL_TIME_STRIDE */>
                    ( new CollPointCostFunctor(sqrtWeight, sizeA, sizeB) );
            }

        protected:
            const CeresScalar               _sqrtWeight;
            const std::array<CeresScalar,3> _sizeA, _sizeB;
            const CeresScalar               _radiusA, _radiusB;
    }; // CollPointCostFunctor

    template <typename _FunctorInfos>
    void restrictCollPoint(CuboidsT const& cuboids, const CollId collId, const Weights& weights, ceres::Problem& problem, PhysIndexer& indexer, _FunctorInfos& costFunctors) {
        using ceres::CostFunction;
        using ceres::CeresScalar;
        //typedef Cuboid::Vector3 Vector3;

        CeresScalar *const collPoint            = indexer.getCollisionPoint      ( collId );
        CeresScalar *const collTime             = indexer.getCollisionTime       ( collId );
        //CeresScalar *const parabolaShared       = indexer.getParabolaSharedParams();
        CeresScalar *const parabolaTranslationA = indexer.getParabolaTranslation ( 0, collId );
        CeresScalar *const parabolaTranslationB = indexer.getParabolaTranslation ( 1, collId );

        CostFunction *const costFunction = CollPointCostFunctor::Create(weights.collPointWeight, cuboids.at(0).getSize(), cuboids.at(1).getSize());
        problem.AddResidualBlock( costFunction, NULL, collPoint, parabolaTranslationA, parabolaTranslationB, collTime );

        char name[255];
        sprintf(name, "CollPointCostFunctor_coll%u", collId);
        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {collPoint, parabolaTranslationA, parabolaTranslationB, collTime}));
    }

  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_RESTRICTCOLLPOINTTERM_HPP
