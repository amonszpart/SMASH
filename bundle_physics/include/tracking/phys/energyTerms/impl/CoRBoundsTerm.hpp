//
// Created by bontius on 02/05/16.
//

#ifndef TRACKVIDEO_PHYS_CORFUNCTOR_HPP
#define TRACKVIDEO_PHYS_CORFUNCTOR_HPP

#include "tracking/phys/energyTerms/CoRBoundsTerm.h"

#include "tracking/phys/inertiaEstimation.hpp"
#include "tracking/phys/energyTerms/impl/impulseTerm.hpp"
#include "tracking/phys/energyTerms/impl/poseTerm.hpp"
#include "tracking/phys/partUtil.h"
#include "tracking/annot/cuboid.h"
#include "ceres/cost_function.h"
#include "ceres/typedefs.h"

namespace tracking {
namespace bundle_physics {

/**
 *  \param[in] sqrtWeight   Sqrt of term weight.
 *  \param[in] dt           Timestep size.
 */
template<typename _Scalar>
CoRCostFunctor::CoRCostFunctor(
    CeresScalar    const sqrtCoRWeight,
    CeresScalar    const sqrtKEWeight,
    CeresScalar    const dt,
    CeresScalar    const startFrame,
    _Scalar const* const sizeA,
    _Scalar const* const sizeB,
    SHAPE          const shapeA,
    SHAPE          const shapeB
) : _sqrtCoRWeight(sqrtCoRWeight),
    _sqrtKEWeight(sqrtKEWeight),
    _dt(dt),
    _fps(1./dt),
    _startFrame(startFrame),
    _sqrSizeA{sizeA[0] * sizeA[0], sizeA[1] * sizeA[1], sizeA[2] * sizeA[2]},
    _sqrSizeB{sizeB[0] * sizeB[0], sizeB[1] * sizeB[1], sizeB[2] * sizeB[2]},
    _shapeA(shapeA),
    _shapeB(shapeB)
{}

/** \brief Limits CoR to be 0..1: \f$ c = ((v'_b - v'_a) . n) / ((v_a - v_b) . n) \f$ and KE to not increase.
 *
 *  \param[in] rotG[2]          \f$ \theta_x, \theta_{y1} \f$
 *  \param[in] a[1]             \f$ b_1 \f$ acceleration
 *  \param[in] freeA0[3]        Pre-collision, first object: \f$ \theta^{a,pre}_{y0}, b^{a,pre}_2, b^{a,pre}_3 \f$ ("rotY0", "b", "s")
 *  \param[in] freeA1[3]        Post-collision, first object: \f$ \theta^{a,post}_{y0}, b^{a,post}_2, b^{a,post}_3 \f$ ("rotY0", "b", "s")
 *  \param[in] freeB0[3]        Pre-collision, second object: \f$ \theta^{b,pre}_{y0}, b^{b,pre}_2, b^{b,pre}_3 \f$ ("rotY0", "b", "s")
 *  \param[in] freeB1[3]        Post-collision, second object: \f$ \theta^{b,post}_{y0}, b^{b,post}_2, b^{b,post}_3 \f$ ("rotY0", "b", "s")
 *  \param[in] translationA[3]  Collision position of objectA:\f$ \mathbf{b}^{a}_4 \f$
 *  \param[in] translationB[3]  Collision position of objectB:\f$ \mathbf{b}^{b}_4 \f$
 *  \param[in] q0A[4]           Collision pose of objectA:\f$ \mathbf{q}^{c,a} \f$
 *  \param[in] q0B[4]           Collision pose of objectB:\f$ \mathbf{q}^{c,b} \f$
 *  \param[in] momentumA0[3]    Pre-collision, first object momentum: \f$ \mathbf{k}^a \f$
 *  \param[in] momentumA1[3]    Post-collision, first object momentum:\f$ \mathbf{k}^{a,post} \f$
 *  \param[in] momentumB0[3]    Pre-collision, second object momentum: \f$ \mathbf{k}^b \f$
 *  \param[in] momentumB1[3]    Post-collision, secondt object momentum:\f$ \mathbf{k}^{b,post} \f$
 *  \param[in] massA[1]         Mass of first object:\f$ m^a \f$ (defaults to fixed 1.)
 *  \param[in] massB[1]         Mass of second object:\f$ m^b \f$
 *  \param[in] collPoint[3]     Relative collision point w.r.t. object A:\f$ \mathbf{x}_c \f$
 *  \param[in] impulse[3]       Impulse vector:\f$ j\mathbf{n} \f$
 *  \param[in] collTime[1]      Time of collision:\f$ t^c \f$ (unused)
 *  \param[out] residuals[2]    Residuals from CoR and KE parts.
 */
template<typename T>
bool CoRCostFunctor::operator()(T const* const* unknowns, T* residuals) const {
    typedef Eigen::Map<const Eigen::Matrix<T,3,1>> _CMapT;

    T const* rotG         = unknowns[0];
    T const* a            = unknowns[1];
    T const* freeA0       = unknowns[2];
    T const* freeA1       = unknowns[3];
    T const* freeB0       = unknowns[4];
    T const* freeB1       = unknowns[5];
    T const* translationA = unknowns[6];
    T const* translationB = unknowns[7];
    T const* q0A          = unknowns[8];
    T const* q0B          = unknowns[9];
    T const* momentumA0   = unknowns[10];
    T const* momentumA1   = unknowns[11];
    T const* momentumB0   = unknowns[12];
    T const* momentumB1   = unknowns[13];
    T const* massA        = unknowns[14];
    T const* massB        = unknowns[15];
    T const* collPoint    = unknowns[16];
    T const* impulse      = unknowns[17];
//                T const* collTime     = unknowns[18];

    T g[3];
    GravityCostFunctor::getWorldGravity(rotG, a, _fps, /* [out] gravity: */ g);

    Eigen::Matrix<T,3,1> vA0, vA1, vB0, vB1;
    LinVelCostFunctor::getLinearVelocity(rotG, freeA0, T(_startFrame), T(_dt), T(_fps), g, vA0.data());
    LinVelCostFunctor::getLinearVelocity(rotG, freeA1, T(_startFrame), T(_dt), T(_fps), g, vA1.data());
    LinVelCostFunctor::getLinearVelocity(rotG, freeB0, T(_startFrame), T(_dt), T(_fps), g, vB0.data());
    LinVelCostFunctor::getLinearVelocity(rotG, freeB1, T(_startFrame), T(_dt), T(_fps), g, vB1.data());

    T invIA[3], invIB[3];
    getInvI(_shapeA, massA, _sqrSizeA.data(), invIA);
    getInvI(_shapeB, massB, _sqrSizeB.data(), invIB);

    T omegaA0[4], omegaA1[4], omegaB0[4], omegaB1[4]; // [w=0, x, y, z]^T
    estimateOmega(q0A,momentumA0,invIA,omegaA0);
    estimateOmega(q0A,momentumA1,invIA,omegaA1);
    estimateOmega(q0B,momentumB0,invIB,omegaB0);
    estimateOmega(q0B,momentumB1,invIB,omegaB1);

    // relative collision point from object B's centroid
    Eigen::Matrix<T,3,1> cpB = _CMapT(translationA) + _CMapT(collPoint) - _CMapT(translationB);

    Eigen::Matrix<T,3,1> radVelA0, radVelA1, radVelB0, radVelB1;
    radVelA0 = _CMapT(omegaA0+1).cross(_CMapT(collPoint)); // omega[0] is the weight, == 0
    radVelA1 = _CMapT(omegaA1+1).cross(_CMapT(collPoint));
    radVelB0 = _CMapT(omegaB0+1).cross(cpB);
    radVelB1 = _CMapT(omegaB1+1).cross(cpB);

    Eigen::Matrix<T,3,1> collNormal = _CMapT(impulse).normalized();

    T relVel1 = (vB1 + radVelB1 - vA1 - radVelA1).dot(collNormal);
    T relVel0 = (vA0 + radVelA0 - vB0 - radVelB0).dot(collNormal);

    // prevent infinite division
    if (relVel0 == T(0.))
        residuals[0] = T(0.);
    else {
        T c = relVel1 / relVel0;
        if (c < T(0.))
            residuals[0] = T(_sqrtCoRWeight) * (T(1.) - c);
        else if (c > T(1.))
            residuals[0] = T(_sqrtCoRWeight) * (c - T(1.));
        else
            residuals[0] = T(0.);
    }

    // energy
    T kinEBefore = T(0.5) * (massA[0] * vA0.dot(vA0) + massB[0] * vB0.dot(vB0));
    kinEBefore += T(0.5) * (T(1.) / invIA[0] * omegaA0[1] * omegaA0[1] +
                            T(1.) / invIB[0] * omegaB0[1] * omegaB0[1] +
                            T(1.) / invIA[1] * omegaA0[2] * omegaA0[2] +
                            T(1.) / invIB[1] * omegaB0[2] * omegaB0[2] +
                            T(1.) / invIA[2] * omegaA0[3] * omegaA0[3] +
                            T(1.) / invIB[2] * omegaB0[3] * omegaB0[3]  );

    T kinEAfter  = T(0.5) * (massA[0] * vA1.dot(vA1) + massB[0] * vB1.dot(vB1));
    kinEAfter += T(0.5) * (T(1.) / invIA[0] * omegaA1[1] * omegaA1[1] +
                           T(1.) / invIB[0] * omegaB1[1] * omegaB1[1] +
                           T(1.) / invIA[1] * omegaA1[2] * omegaA1[2] +
                           T(1.) / invIB[1] * omegaB1[2] * omegaB1[2] +
                           T(1.) / invIA[2] * omegaA1[3] * omegaA1[3] +
                           T(1.) / invIB[2] * omegaB1[3] * omegaB1[3]  );
    if (kinEAfter > kinEBefore) {
        residuals[1] = T(_sqrtKEWeight) * (kinEAfter  - kinEBefore);
    } else {
        residuals[1] = T(0.);
    }

    return true;
} //...operator()

template <typename _Scalar>
ceres::CostFunction* CoRCostFunctor::Create(
    CeresScalar    const sqrtCoRWeight,
    CeresScalar    const sqrtKEWeight,
    CeresScalar    const dt,
    CeresScalar    const startFrame,
    _Scalar const* const sizeA,
    _Scalar const* const sizeB,
    SHAPE          const shapeA,
    SHAPE          const shapeB)
{
    auto fn = new ceres::DynamicAutoDiffCostFunction<CoRCostFunctor>(
        new CoRCostFunctor(sqrtCoRWeight,sqrtKEWeight,dt,startFrame,sizeA,sizeB,shapeA,shapeB)
    );
    fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_A_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_TRANSLATION_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_TRANSLATION_STRIDE);
    fn->AddParameterBlock(PhysIndexer::POSE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::POSE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MASS_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MASS_STRIDE);
    fn->AddParameterBlock(PhysIndexer::COLL_POINTS_STRIDE);
    fn->AddParameterBlock(PhysIndexer::IMPULSE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::COLL_TIME_STRIDE);

    fn->SetNumResiduals(NUM_RESIDUALS);
    return fn;
}

template <typename _FunctorInfosT>
void addCorFunctor(
    ceres::Problem             & problem,
    PhysIndexer                & indexer,
    _FunctorInfosT             & costFunctors,
    FrameIdsT             const& frameIds,
    Weights               const& weights,
    Cuboid                const& cuboid0, // assumed to have id 0 in indexer
    Cuboid                const& cuboid1, // assumed to have id 1 in indexer
    Consts                const& consts) {
    using ceres::CeresScalar;
    using ceres::CostFunction;

    const CuboidId objA = 0;
    const CuboidId objB = 1;
    char name[255];
    for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
        const int partIdBefore = getPartIdBefore(collId);
        const int partIdAfter  = getPartIdAfter (collId);

        CeresScalar *const massA                = indexer.getMass(objA);
        CeresScalar *const massB                = indexer.getMass(objB);
        CeresScalar *const parabolaRotShared    = indexer.getParabolaRotationShared();
        CeresScalar *const parabolaA            = indexer.getParabolaSquaredParam();
        CeresScalar *const parabolaFreeA0       = indexer.getParabolaFreeParams(objA, partIdBefore);
        CeresScalar *const parabolaFreeA1       = indexer.getParabolaFreeParams(objA, partIdAfter);
        CeresScalar *const parabolaFreeB0       = indexer.getParabolaFreeParams(objB, partIdBefore);
        CeresScalar *const parabolaFreeB1       = indexer.getParabolaFreeParams(objB, partIdAfter);
        CeresScalar *const parabolaTranslationA = indexer.getParabolaTranslation(objA, collId);
        CeresScalar *const parabolaTranslationB = indexer.getParabolaTranslation(objB, collId);
        CeresScalar *const poseA                = indexer.getCollisionPose(objA, collId);
        CeresScalar *const poseB                = indexer.getCollisionPose(objB, collId);
        CeresScalar *const momentumA0           = indexer.getMomentum(objA, partIdBefore);
        CeresScalar *const momentumA1           = indexer.getMomentum(objA, partIdAfter);
        CeresScalar *const momentumB0           = indexer.getMomentum(objB, partIdBefore);
        CeresScalar *const momentumB1           = indexer.getMomentum(objB, partIdAfter);
        CeresScalar *const collisionTime        = indexer.getCollisionTime(collId);
        CeresScalar *const collisionPoint       = indexer.getCollisionPoint(collId);
        CeresScalar *const impulse              = indexer.getImpulse(collId);

        CostFunction* costFunction = CoRCostFunctor::Create(weights.corWeight, weights.keWeight, consts.k_dt,
                                                            frameIds.front(), cuboid0.getSize().data(),
                                                            cuboid1.getSize().data(), cuboid0.getShape(),
                                                            cuboid1.getShape());
        std::vector<CeresScalar*> unknowns;
        unknowns.push_back(parabolaRotShared);
        unknowns.push_back(parabolaA);
        unknowns.push_back(parabolaFreeA0);
        unknowns.push_back(parabolaFreeA1);
        unknowns.push_back(parabolaFreeB0);
        unknowns.push_back(parabolaFreeB1);
        unknowns.push_back(parabolaTranslationA);
        unknowns.push_back(parabolaTranslationB);
        unknowns.push_back(poseA);
        unknowns.push_back(poseB);
        unknowns.push_back(momentumA0);
        unknowns.push_back(momentumA1);
        unknowns.push_back(momentumB0);
        unknowns.push_back(momentumB1);
        unknowns.push_back(massA);
        unknowns.push_back(massB);
        unknowns.push_back(collisionPoint);
        unknowns.push_back(impulse);
        unknowns.push_back(collisionTime);

        problem.AddResidualBlock(costFunction, NULL, unknowns);
        sprintf(name, "CoR_coll%u", collId);
        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, unknowns));
    } //...for collisions
} //...addCorFunctor()


// ==================================================================================================================//
// InfMass //
// ==================================================================================================================//

struct CoRCostFunctorInfMass {
    public:
        using CeresScalar = ceres::CeresScalar;
        enum { NUM_RESIDUALS = 2 };

        /**
         *  \param[in] sqrtWeight   Sqrt of term weight.
         *  \param[in] dt           Timestep size.
         */
        template <typename _Scalar>
        CoRCostFunctorInfMass(
            CeresScalar    const sqrtCoRWeight,
            CeresScalar    const sqrtKEWeight,
            CeresScalar    const dt,
            CeresScalar    const startFrame,
            _Scalar const* const sizeA,
            SHAPE          const shapeA);

        /** \brief Estimates CoR: c = ((v'_b - v'_a) . n) / ((v_a - v_b) . n). */
        template <typename T>
        bool operator()(T const* const* unknowns, T* residuals) const;

        // Factory to hide the construction of the CostFunction object from the client code.
        template <typename _Scalar>
        static ceres::CostFunction *Create(
            CeresScalar    const sqrtCoRWeight,
            CeresScalar    const sqrtKEWeight,
            CeresScalar    const dt,
            CeresScalar    const startFrame,
            _Scalar const* const sizeA,
            SHAPE          const shapeA);

    protected:
        CeresScalar                const _sqrtCoRWeight;
        CeresScalar                const _sqrtKEWeight;
        CeresScalar                const _dt;               //!< Time between frames (1/fps).
        CeresScalar                const _fps;              //!< Frames per sec (1/dt).
        CeresScalar                const _startFrame;
        std::array<CeresScalar, 3> const _sqrSizeA; //!< Size of the object, used for inertia tensor estimation
        SHAPE                      const _shapeA;
}; //...CoRCostFunctor

/**
 *  \param[in] sqrtWeight   Sqrt of term weight.
 *  \param[in] dt           Timestep size.
 */
template <typename _Scalar>
CoRCostFunctorInfMass::CoRCostFunctorInfMass(
    CeresScalar    const sqrtCoRWeight,
    CeresScalar    const sqrtKEWeight,
    CeresScalar    const dt,
    CeresScalar    const startFrame,
    _Scalar const* const sizeA,
    SHAPE          const shapeA
) : _sqrtCoRWeight(sqrtCoRWeight),
    _sqrtKEWeight(sqrtKEWeight),
    _dt(dt),
    _fps(1./dt),
    _startFrame(startFrame),
    _sqrSizeA{sizeA[0] * sizeA[0], sizeA[1] * sizeA[1], sizeA[2] * sizeA[2]},
    _shapeA(shapeA)
{}

/** \brief Limits CoR to be 0..1: \f$ c = ((v'_b - v'_a) . n) / ((v_a - v_b) . n) \f$ and KE to not increase.
 *
 *  \param[in] rotG[2]          \f$ \theta_x, \theta_{y1} \f$
 *  \param[in] a[1]             \f$ b_1 \f$ acceleration
 *  \param[in] freeA0[3]        Pre-collision, first object: \f$ \theta^{a,pre}_{y0}, b^{a,pre}_2, b^{a,pre}_3 \f$ ("rotY0", "b", "s")
 *  \param[in] freeA1[3]        Post-collision, first object: \f$ \theta^{a,post}_{y0}, b^{a,post}_2, b^{a,post}_3 \f$ ("rotY0", "b", "s")
 *  \param[in] translationA[3]  Collision position of objectA:\f$ \mathbf{b}^{a}_4 \f$
 *  \param[in] q0A[4]           Collision pose of objectA:\f$ \mathbf{q}^{c,a} \f$
 *  \param[in] momentumA0[3]    Pre-collision, first object momentum: \f$ \mathbf{k}^a \f$
 *  \param[in] momentumA1[3]    Post-collision, first object momentum:\f$ \mathbf{k}^{a,post} \f$
 *  \param[in] massA[1]         Mass of first object:\f$ m^a \f$ (defaults to fixed 1.)
 *  \param[in] collPoint[3]     Relative collision point w.r.t. object A:\f$ \mathbf{x}_c \f$
 *  \param[in] impulse[3]       Impulse vector:\f$ j\mathbf{n} \f$
 *  \param[in] collTime[1]      Time of collision:\f$ t^c \f$ (unused)
 *  \param[out] residuals[2]    Residuals from CoR and KE parts.
 */
template<typename T>
bool CoRCostFunctorInfMass::operator()(T const* const* unknowns, T* residuals) const {
    typedef Eigen::Map<const Eigen::Matrix<T,3,1>> _CMapT;

    T const* rotG         = unknowns[0];
    T const* a            = unknowns[1];
    T const* freeA0       = unknowns[2];
    T const* freeA1       = unknowns[3];
    //T const* freeB0       = unknowns[4];
    //T const* freeB1       = unknowns[5];
    //T const* translationA = unknowns[4];
    //T const* translationB = unknowns[7];
    T const* q0A          = unknowns[5];
    //T const* q0B          = unknowns[9];
    T const* momentumA0   = unknowns[6];
    T const* momentumA1   = unknowns[7];
    //T const* momentumB0   = unknowns[12];
    //T const* momentumB1   = unknowns[13];
    T const* massA        = unknowns[8];
    //T const* massB        = unknowns[15];
    T const* collPoint    = unknowns[9];
    T const* impulse      = unknowns[10];
//                T const* collTime     = unknowns[18];

    T g[3];
    GravityCostFunctor::getWorldGravity(rotG, a, _fps, /* [out] gravity: */ g);

    Eigen::Matrix<T,3,1> vA0, vA1;
    LinVelCostFunctor::getLinearVelocity(rotG, freeA0, T(_startFrame), T(_dt), T(_fps), g, vA0.data());
    LinVelCostFunctor::getLinearVelocity(rotG, freeA1, T(_startFrame), T(_dt), T(_fps), g, vA1.data());

    T invIA[3];
    getInvI(_shapeA, massA, _sqrSizeA.data(), invIA);

    T omegaA0[4], omegaA1[4]; // [w=0, x, y, z]^T
    estimateOmega(q0A,momentumA0,invIA,omegaA0);
    estimateOmega(q0A,momentumA1,invIA,omegaA1);

    Eigen::Matrix<T,3,1> radVelA0, radVelA1;
    radVelA0 = _CMapT(omegaA0+1).cross(_CMapT(collPoint)); // omega[0] is the weight, == 0
    radVelA1 = _CMapT(omegaA1+1).cross(_CMapT(collPoint));

    Eigen::Matrix<T,3,1> collNormal = _CMapT(impulse).normalized();

    T relVel1 = (-vA1 -radVelA1).dot(collNormal);
    T relVel0 = (vA0 + radVelA0).dot(collNormal);

    // prevent infinite division
    if (relVel0 == T(0.))
        residuals[0] = T(0.);
    else {
        T c = relVel1 / relVel0;
        if (c < T(0.))
            residuals[0] = T(_sqrtCoRWeight) * (T(1.) - c);
        else if (c > T(1.))
            residuals[0] = T(_sqrtCoRWeight) * (c - T(1.));
        else
            residuals[0] = T(0.);
    }

    // energy
    T kinEBefore = T(0.5) * (massA[0] * vA0.dot(vA0));
    kinEBefore += T(0.5) * (T(1.) / invIA[0] * omegaA0[1] * omegaA0[1] +
                            T(1.) / invIA[1] * omegaA0[2] * omegaA0[2] +
                            T(1.) / invIA[2] * omegaA0[3] * omegaA0[3] );


    T kinEAfter  = T(0.5) * (massA[0] * vA1.dot(vA1));
    kinEAfter += T(0.5) * (T(1.) / invIA[0] * omegaA1[1] * omegaA1[1] +
                           T(1.) / invIA[1] * omegaA1[2] * omegaA1[2] +
                           T(1.) / invIA[2] * omegaA1[3] * omegaA1[3] );
    if (kinEAfter > kinEBefore) {
        residuals[1] = T(_sqrtKEWeight) * (kinEAfter  - kinEBefore);
    } else {
        residuals[1] = T(0.);
    }

    return true;
} //...operator()

// Factory to hide the construction of the CostFunction object from the client code.
template <typename _Scalar>
ceres::CostFunction* CoRCostFunctorInfMass::Create(
    CeresScalar    const sqrtCoRWeight,
    CeresScalar    const sqrtKEWeight,
    CeresScalar    const dt,
    CeresScalar    const startFrame,
    _Scalar const* const sizeA,
    SHAPE          const shapeA)
{
    auto fn = new ceres::DynamicAutoDiffCostFunction<CoRCostFunctorInfMass>{
        new CoRCostFunctorInfMass{sqrtCoRWeight, sqrtKEWeight, dt, startFrame, sizeA, shapeA}};

    fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_A_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
    //fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
    //fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::PARABOLA_TRANSLATION_STRIDE);
    //fn->AddParameterBlock(PhysIndexer::PARABOLA_TRANSLATION_STRIDE);
    fn->AddParameterBlock(PhysIndexer::POSE_STRIDE);
    //fn->AddParameterBlock(PhysIndexer::POSE_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
    //fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
    //fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
    fn->AddParameterBlock(PhysIndexer::MASS_STRIDE);
    //fn->AddParameterBlock(PhysIndexer::MASS_STRIDE);
    fn->AddParameterBlock(PhysIndexer::COLL_POINTS_STRIDE);
    fn->AddParameterBlock(PhysIndexer::IMPULSE_STRIDE);

    fn->SetNumResiduals(NUM_RESIDUALS);
    return fn;
}

template <typename _FunctorInfosT>
void addCorFunctorInfMass(
    ceres::Problem             & problem,
    PhysIndexer                & indexer,
    _FunctorInfosT             & costFunctors,
    FrameIdsT             const& frameIds,
    Weights               const& weights,
    Cuboid                const& cuboid0, // assumed to have id 0 in indexer
    Consts                const& consts) {
    using ceres::CeresScalar;
    using ceres::CostFunction;

    const CuboidId objA = 0;
    //const CuboidId objB = 1;
    char name[255];
    for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
        const int partIdBefore = getPartIdBefore(collId);
        const int partIdAfter  = getPartIdAfter (collId);

        CeresScalar *const massA                = indexer.getMass(objA);
        //CeresScalar *const massB                = indexer.getMass(objB);
        CeresScalar *const parabolaRotShared    = indexer.getParabolaRotationShared();
        CeresScalar *const parabolaA            = indexer.getParabolaSquaredParam();
        CeresScalar *const parabolaFreeA0       = indexer.getParabolaFreeParams(objA, partIdBefore);
        CeresScalar *const parabolaFreeA1       = indexer.getParabolaFreeParams(objA, partIdAfter);
        //CeresScalar *const parabolaFreeB0       = indexer.getParabolaFreeParams(objB, partIdBefore);
        //CeresScalar *const parabolaFreeB1       = indexer.getParabolaFreeParams(objB, partIdAfter);
        CeresScalar *const parabolaTranslationA = indexer.getParabolaTranslation(objA, collId);
        //CeresScalar *const parabolaTranslationB = indexer.getParabolaTranslation(objB, collId);
        CeresScalar *const poseA                = indexer.getCollisionPose(objA, collId);
        //CeresScalar *const poseB                = indexer.getCollisionPose(objB, collId);
        CeresScalar *const momentumA0           = indexer.getMomentum(objA, partIdBefore);
        CeresScalar *const momentumA1           = indexer.getMomentum(objA, partIdAfter);
        //CeresScalar *const momentumB0           = indexer.getMomentum(objB, partIdBefore);
        //CeresScalar *const momentumB1           = indexer.getMomentum(objB, partIdAfter);
        CeresScalar *const collisionPoint       = indexer.getCollisionPoint(collId);
        CeresScalar *const impulse              = indexer.getImpulse(collId);

        CostFunction* costFunction = CoRCostFunctorInfMass::Create(weights.corWeight, weights.keWeight, consts.k_dt,
                                                            frameIds.front(), cuboid0.getSize().data(),
                                                            cuboid0.getShape());
        std::vector<CeresScalar*> unknowns;
        unknowns.push_back(parabolaRotShared);
        unknowns.push_back(parabolaA);
        unknowns.push_back(parabolaFreeA0);
        unknowns.push_back(parabolaFreeA1);
        //unknowns.push_back(parabolaFreeB0);
        //unknowns.push_back(parabolaFreeB1);
        unknowns.push_back(parabolaTranslationA);
        //unknowns.push_back(parabolaTranslationB);
        unknowns.push_back(poseA);
        //unknowns.push_back(poseB);
        unknowns.push_back(momentumA0);
        unknowns.push_back(momentumA1);
        //unknowns.push_back(momentumB0);
        //unknowns.push_back(momentumB1);
        unknowns.push_back(massA);
        //unknowns.push_back(massB);
        unknowns.push_back(collisionPoint);
        unknowns.push_back(impulse);

        problem.AddResidualBlock(costFunction, NULL, unknowns);
        sprintf(name, "CoRInfMass_coll%u", collId);
        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, unknowns));
    } //...for collisions
} //...addCorFunctor()


} //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_CORFUNCTOR_HPP
