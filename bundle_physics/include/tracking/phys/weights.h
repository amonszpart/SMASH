//
// Created by bontius on 04/01/16.
//

#ifndef TRACKVIDEO_WEIGHTS_H
#define TRACKVIDEO_WEIGHTS_H

#include "tracking/common/eigen.h"
#include "tracking/common/util/exception.h"

namespace tracking {
  namespace bundle_physics {

    struct Bound1D
    {
        public:
            Bound1D(double lower, double upper)
                : lower(lower), upper(upper) {}
        public:
            double lower, upper;
    };

    struct Bound
    {
        public:
            Bound(Eigen::Vector3d const& lower, Eigen::Vector3d const& upper)
                : lower(lower), upper(upper) { }
        public:
            Eigen::Vector3d lower, upper;

        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

    struct Bounds
    {
        public:

            Bounds()
                : momentum      ({{-0.2,-0.2,-0.2},{0.2,0.2,0.2}}),
                translation     ({{-4.,-4.,0.},{4.,4.,10.}}),
                parabolaRotation({{-720.,-720.,-720.},{720.,720.,720.}}),
                parabolaB       ({-0.2,0.2}),
                parabolaS       ({-0.2,0.2}),
                mass            ({0.1,5.}),
                objectSize      ({-2.,-2.,-2.},{2.,2.,2.}),
                collTime        ({-1.,-1.})
            {}

            Bound   momentum,
                    translation;
            Bound   parabolaRotation; //!< free: y0, shared: x, y1
            Bound1D parabolaB,
                    parabolaS;
            Bound1D mass;
            Bound   objectSize;       //!< Object size used for collision point and pointTerms.
            Bound1D collTime;         //!< Absolute limit of the input collTime
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
    DEFINE_EXCEPTION( Weights_InitDiscrepancy )
    struct Weights
    {
        Weights()
            : targetNormalization(10.),
              observedPosWeight(std::sqrt(10.)),
              observedPoseWeight(1.),
              velocityWeight(std::sqrt(10.)),
              conservationWeight(std::sqrt(10.)),
              gravityDownWeight(1.), corWeight(1.), keWeight(1.),
              sizePriorWeight(1.), repulseWeight(1.),
              collPointWeight(0.), pointsUvWeight(0.), minMomentumWeight(0.), compactnessWeight(0.), pointHuberParam(0.),
              compactnessHuberParam(0.), fps(240.), poseIntegralSteps(1),
              doVis(true), solveFlags(SOLVE_FLAGS::SOLVE),
              solveStages(SOLVE_STAGES::SOLVE_COUPLED | SOLVE_STAGES::SOLVE_FREE_CP),
              initFlags(INIT_FLAGS::NONE), fixFlags(FIX_FLAGS::FIX_NONE),
              max_num_iterations(std::numeric_limits<int>::max()), max_solver_time_in_seconds(0.),
              function_tolerance(1.e-12), parameter_tolerance(1.e-12)
        {}

        enum SOLVE_FLAGS {
            SOLVE      = 0x01,
            SOLVE_POS  = 0x02,
            NO_SOLVE   = 0x04,
            FIX_POINTS = 0x08,
            RE_SOLVE   = 0x10
        };
        enum SOLVE_STAGES {
            SOLVE_MOMENTA = 0x01, //!< solve for momenta, without coupling (first stage)
            SOLVE_COUPLED = 0x02, //!< solve full with fixed collision point
            SOLVE_FREE_CP = 0x04, //!< solve with free collision point
            SOLVE_FREE_MASS = 0x08  //!< solve with free mass
        };
        enum INIT_FLAGS {
            NONE                   = 0x00,
            USE_INPUT_MOMENTUM     = 0x01,
            USE_INPUT_POSES        = 0x02,
            USE_INPUT_COLLTIMES    = 0x04,
            USE_INPUT_PARABOLAS    = 0x08,
            USE_INPUT_TRANSLATIONS = 0x10,
            USE_INPUT_COLLPOINTS   = 0x20,
            USE_INPUT_MASS         = 0x40
        };
        enum FIX_FLAGS {
            FIX_NONE          = 0x00,
            FIX_GRAVITY       = 0x01,
            FIX_COLLTIMES     = 0x02,
            FIX_MOMENTA       = 0x04,
            FIX_PARABOLAS     = 0x08,
            FIX_POSE          = 0x10,
            FIX_COLLPOINT     = 0x20
            //FIX_GRAVITY_AND_A = 0x01,
        };

        /** @name Weight parameters
         *  @{
         */
        double targetNormalization; //!< Term sum weight is scaled by this for \ref PointCostFunctor and \ref CompactnessCostFunctor.

        double observedPosWeight;       //!< Observed positions (parabola fit)
        double observedPoseWeight;      //!< Observed poses
        double velocityWeight;          //!< \f$ va' = va + Jn/Ma, vb' = vb + Jn/Mb \f$, and angular impulse (used twice).
        double conservationWeight;      //!< Conservation of linear and angular momentum separately (used twice).
        double gravityDownWeight;       //!< Gravity vector scale
        double corWeight;               //!< Weight of coefficient of restitution bounds.
        double keWeight;                //!< Kinetic energy should not increase.

        double sizePriorWeight;         //!< Used in position initialization. Connects blob-size to object size.
        double repulseWeight;           //!< Weight in position initialization to prevent 2d parabola intersection.

        double collPointWeight;         //!< Collision point should be close to centroid line
        double pointsUvWeight;          //!< Reprojection fidelity.
        double minMomentumWeight;       //!< Minimal momentum solution.
        double compactnessWeight;       //!< Weight on compactness prior used in \ref CompactnessCostFunctor.
        double pointHuberParam;         //!< HuberLoss parameter for point term. SquaredLoss is used, if < 0.
        double compactnessHuberParam;   //!< HuberLoss parameter for compactness term. SquaredLoss is used, if < 0.
        /** @} */

        /** @name Constants
         *  @{
         */
        double fps;                     //!< Video speed (frames per sec).
        float  poseIntegralSteps;       //!< Integration granularity. //TODO: determine unit (per timestep, or overall?)
        Bounds bounds;                  //!< Variable bounds
        /** @} */

        /** @name Debugg parameters
         *  @{
         */
        bool   doVis;                   //!< Temporary debug flag
        int    solveFlags;              //!< Which terms to optimize for
        int    solveStages;             //!< Which solution steps to use
        /** @} */

        /** @name Optimization strategy
         *  @{
         */
        int    initFlags;               //!< Which parts of initialization to use
        int    fixFlags;                //!< Which variables are not optimized.
        /** @} */

        /** @name Optimization parameters (ceres)
         *  @{
         */
        int    max_num_iterations;         //!< Stop after this many iterations.
        double max_solver_time_in_seconds; //!< Stop after this many seconds.
        double function_tolerance;         //!< Minimizer terminates when (new_cost - old_cost) < function_tolerance * old_cost.
        double parameter_tolerance;        //!< Minimizer terminates when |step|_2 <= parameter_tolerance * ( |x|_2 +  parameter_tolerance).
        /**@}*/
    };
  }//...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_WEIGHTS_H
