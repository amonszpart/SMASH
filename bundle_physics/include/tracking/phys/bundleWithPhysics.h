#ifndef TV_BUNDLEWITHVELOCITY_H
#define TV_BUNDLEWITHVELOCITY_H

#include "tracking/phys/physIndexer.h"
#include "tracking/phys/typedefs.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/consts.h"
#include "tracking/phys/weights.h"
#include "tracking/annot/cuboid.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/util/exception.h"
#include <map>

namespace tracking {
class Mapper;
namespace bundle_physics {
DEFINE_EXCEPTION(BundleWithPhysics_CeresTerminationFailure)
DEFINE_EXCEPTION(BundleWithPhysics_AnimateCuboids_InitialCuboidsDontHaveEnoughCuboidsForCurrentCuboidIt)
DEFINE_EXCEPTION(BundleWithPhysics_MassNotOne) // it should  be ok, just not zero
DEFINE_EXCEPTION(BundleWithPhysics_InitialCollisionTimeFixedButNotProvided)
DEFINE_EXCEPTION(BundleWithPhysics_AnimateCuboids_SubstepsHaveToBeOdd)
DEFINE_EXCEPTION(BundleWithPhysics_InitPoses_NoClosestPoses)
DEFINE_EXCEPTION(BundleWithPhysics_InitPoses_NoInitProvided)
DEFINE_EXCEPTION(BundleWithPhysics_InitPoses_ASingleCollisionAssumed)
DEFINE_EXCEPTION(BundleWithPhysics_InitPoses_CollTimeNotBetweenObservations)
DEFINE_EXCEPTION(BundleWithPhysics_InitMomentum_UseInputMomentumNotImplemented)
DEFINE_EXCEPTION(BundleWithPhysics_EstimateC_TwoCuboidsAssumed)
DEFINE_EXCEPTION(BundleWithPhysics_OutputDiscrepancy)
DEFINE_EXCEPTION(BundleWithPhysics_GravityDiscrepancy)
DEFINE_EXCEPTION(BundleWithPhysics_OutputParabolaAlreadyExists)
DEFINE_EXCEPTION(BundleWithPhysics_OutputMomentumAlreadyExists)
DEFINE_EXCEPTION(BundleWithPhysics_Output_ShapeMismatch)

class PhysProblem;

class BundleWithPhysics {
    public:
        //typedef Vector3     Vector3;
        //typedef Cuboid::QuaternionT QuaternionT;
        typedef ceres::CeresScalar  CeresScalar;

        static int animateCuboids2(
            BundleWithPhysicsResult      &       out,
            CuboidsT                const&       cuboids,
            GroupedTracks2d         const&       tracks2d,
            //LinRgbsT                const&       rgbs,
            FrameIdsT               const&       frameIds,
            Mapper                  const&       mapper,
            Weights                 const&       weights,
            BundleWithPhysicsResult const* const initial            = nullptr,
            bool                    const        use2dParabolaTerms = true,
            std::string             const        showFlags          = "11111"
        );

        static PhysProblem setup(
            CuboidsT                const&       cuboids,
            GroupedTracks2d         const&       tracks2d,
            //LinRgbsT                const&       rgbs,
            FrameIdsT               const&       frameIds,
            Mapper                  const&       mapper,
            Weights                 const&       weights,
            BundleWithPhysicsResult const* const initial            = nullptr,
            bool                    const        use2dParabolaTerms = true,
            std::shared_ptr<ceres::Problem>      problem            = nullptr
        );

        static BundleWithPhysicsResult solve(
            ceres::Problem                   &problem,
            PhysIndexer                      &indexer,
            ceres::FunctorInfosT             &costFunctors,
            CuboidsT                    const& cuboids,
            FrameIdsT                   const& frameIds,
            Weights                     const& weights,
            BundleWithPhysicsResult     const* const initial,
            Consts                      const& consts,
            std::vector<CuboidId>       const& participants,
            ceres::LossFunctionWrapper       *const poseLoss
            );

        template<typename _Scalar>
        void qToRotationMatrix(Eigen::Matrix<_Scalar, 3, 3> &R, const ceres::CeresScalar q[4]);

        static ceres::Solver::Options getSolverOptions();

//            int initPose(
//                      PhysIndexer        & indexer,
//                const Cuboid             & cuboid,
//                Cuboid::StatesT::iterator& lower,
//                Cuboid::StatesT::iterator& upper,
//                const int                  cuboidId,
//                const FrameId              frameId,
//                const BundleWithPhysicsResult *const initial );

}; //...cls BundleWithPhysics
} //...ns bundle_physics
} //...ns tracking

#endif // TV_BUNDLEWITHVELOCITY_H
