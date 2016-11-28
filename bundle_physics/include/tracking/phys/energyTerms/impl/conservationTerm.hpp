//
// Created by bontius on 03/04/16.
//

#ifndef TRACKVIDEO_PHYS_CONSERVATIONCOSTFUNCTORS_H
#define TRACKVIDEO_PHYS_CONSERVATIONCOSTFUNCTORS_H

#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"
#include "tracking/phys/energyTerms/impl/impulseTerm.hpp"
#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "ceres/ceresUtil.h"
#include "ceres/typedefs.h"

namespace tracking {
namespace bundle_physics {

/** \brief Enforces \f$ \mathbf{v}_a M_a + \mathbf{v}_b M_b = \mathbf{v}'_a M_a + \mathbf{v}'_b M_b \f$ at the time of collision. */
struct ConservationOfLinearMomentum {
    public:
        using CeresScalar = ceres::CeresScalar;

        enum { NUM_RESIDUALS = 3 };

        /**
         *  \param[in] sqrtWeight   Sqrt of term weight.
         *  \param[in] dt           Timestep size.
         *  \param[in] fps          Recording speed (\f$ \frac{1}{dt} \f$).
         *  \param[in] startFrameId Anchor time point for all parabolas.
         */
        ConservationOfLinearMomentum(const CeresScalar sqrtWeight, const CeresScalar fps, const CeresScalar dt, const CeresScalar startFrameId)
            : _sqrtWeight(sqrtWeight), _dt(dt), _fps(fps), _startFrameId(startFrameId) { }

        template<typename T>
        static inline void getLinMomentum(const T mass[1], const T velocity[3], T linMomentum[3])
        {
            linMomentum[0] = mass[0] * velocity[0];
            linMomentum[1] = mass[0] * velocity[1];
            linMomentum[2] = mass[0] * velocity[2];
        }

        /** \brief Enforces \f$ \mathbf{v}_a M_a + \mathbf{v}_b M_b = \mathbf{v}'_a M_a + \mathbf{v}'_b M_b \f$ at the time of collision \p collTime.
         * \param[in] massA Mass of first object.
         * \param[in] massB Mass of second object.
         * \param[in] shared Shared parabola parameters (Two rotation angles \f$ \theta_X \f$ and \f$ \theta_Y \f$ and squared coefficient \f$ a \f$).
         * \param[in] freeA0 Parabola parameters of first object before collision.
         * \param[in] freeA1 Parabola parameters of first object after collision.
         * \param[in] freeB0 Parabola parameters of second object before collision.
         * \param[in] freeB1 Parabola parameters of second object after collision.
         * \param[in] collTime Exact, floating point time point of collision.
         */
        template<typename T>
        bool operator()(const T massA[1], const T massB[1], const T rotG[2], const  T a[1], const T *freeA0, const T *freeA1, const T *freeB0, const T *freeB1,/* const T collTime[1],*/ T *residuals) const
        {
            using ceres::printJet;
            using ceres::printJetVector3;

            T g[3];
            GravityCostFunctor::getWorldGravity( rotG, a, _fps, g );

            T vA0[3], vA1[3], vB0[3], vB1[3];
            LinVelCostFunctor::getLinearVelocity(rotG, freeA0, T(_startFrameId), T(_dt), T(_fps), g, vA0);
            //std::cout << "vA0: " << printJetVector3( vA0 ) << " * " << massA[0] << std::endl;
            LinVelCostFunctor::getLinearVelocity(rotG, freeA1, T(_startFrameId), T(_dt), T(_fps), g, vA1);
            //std::cout << "vA1: " << printJetVector3( vA1 ) << " * " << massA[0] << std::endl;
            LinVelCostFunctor::getLinearVelocity(rotG, freeB0, T(_startFrameId), T(_dt), T(_fps), g, vB0);
            //std::cout << "vB0: " << printJetVector3( vB0 ) << " * " << massB[0] << std::endl;
            LinVelCostFunctor::getLinearVelocity(rotG, freeB1, T(_startFrameId), T(_dt), T(_fps), g, vB1);
            //std::cout << "vB1: " << printJetVector3( vB1 ) << " * " << massB[0] << std::endl;

            T linMomA0[3], linMomA1[3], linMomB0[3], linMomB1[3];
            getLinMomentum(massA, vA0, linMomA0);
            getLinMomentum(massA, vA1, linMomA1);
            getLinMomentum(massB, vB0, linMomB0);
            getLinMomentum(massB, vB1, linMomB1);
            T before[3];
            before[0] = linMomA0[0] + linMomB0[0];
            before[1] = linMomA0[1] + linMomB0[1];
            before[2] = linMomA0[2] + linMomB0[2];
            T after[3];
            after[0] = linMomA1[0] + linMomB1[0];
            after[1] = linMomA1[1] + linMomB1[1];
            after[2] = linMomA1[2] + linMomB1[2];

            residuals[0] = T(_sqrtWeight) * (before[0] - after[0]);
            residuals[1] = T(_sqrtWeight) * (before[1] - after[1]);
            residuals[2] = T(_sqrtWeight) * (before[2] - after[2]);
            //std::cout << "linMomBefore: " << printJet(before[0]) << "," << printJet(before[1]) << "," << printJet(before[2]) << std::endl;
            //std::cout << "linMomAfter: "  << printJet(after[0])  << "," << printJet(after[1])  << "," << printJet(after[2])  << std::endl;


            return true;
        } //...operator()

        // Factory to hide the construction of the CostFunction object from the client code.
        static ceres::CostFunction *Create(const CeresScalar sqrtWeight, const CeresScalar fps, const CeresScalar dt, const CeresScalar startFrameId)
        {
            return (new ceres::AutoDiffCostFunction<ConservationOfLinearMomentum, NUM_RESIDUALS,
                                                    PhysIndexer::MASS_STRIDE, PhysIndexer::MASS_STRIDE, PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE, PhysIndexer::PARABOLA_SHARED_A_STRIDE,
                                                    PhysIndexer::PARABOLA_FREE_STRIDE, PhysIndexer::PARABOLA_FREE_STRIDE, PhysIndexer::PARABOLA_FREE_STRIDE, PhysIndexer::PARABOLA_FREE_STRIDE/*,PhysIndexer::COLL_TIME_STRIDE*/>(
                new ConservationOfLinearMomentum(sqrtWeight, fps, dt, startFrameId)));
        }

    protected:
        const CeresScalar _sqrtWeight;       //!< Sqrt of term weight.
        const CeresScalar _dt;               //!< Timestep per frame.
        const CeresScalar _fps;              //!< Recording speed.
        const CeresScalar _startFrameId;     //!< Time, when the parabola transformations are applied.
}; //...struct ConservationOfLinearMomentum

/** \brief Enforces \f$ \mathbf{c}_a \times (\mathbf{v}_a M_a)  + \mathbf{c}_b \times (\mathbf{v}_b M_b) + \mathbf{L}_a + \mathbf{L}_b =
 *                      \mathbf{c}_a \times (\mathbf{v}'_a M_a) + \mathbf{c}_b \times (\mathbf{v}'_b M_b) + \mathbf{L}'_a + \mathbf{L}'_b \f$ at the time of collision.
 */
struct ConservationOfAngularMomentum {
        using CeresScalar = ceres::CeresScalar;
        enum { NUM_RESIDUALS = 3 };

        /**
         *  \param[in] sqrtWeight   Sqrt of term weight.
         *  \param[in] dt           Timestep size.
         *  \param[in] fps          Recording speed (\f$ \frac{1}{dt} \f$).
         *  \param[in] startFrameId Anchor time point for all parabolas.
         */
        ConservationOfAngularMomentum(const CeresScalar sqrtWeight, const CeresScalar fps, const CeresScalar dt, const CeresScalar startFrameId)
            : _sqrtWeight(sqrtWeight), _dt(dt), _fps(fps), _startFrameId(startFrameId) { }

        template<typename T>
        bool operator()(T const* const *unknowns, T *residuals) const {
            using ceres::printJet;
            using ceres::printJetVector3;

            const T *const massA        = unknowns[0];
            const T *const massB        = unknowns[1];
            const T *const rotG         = unknowns[2];
            const T *const a            = unknowns[3];
            const T *const translationA = unknowns[4];
            const T *const freeA0       = unknowns[5];
            const T *const freeA1       = unknowns[6];
            const T *const translationB = unknowns[7];
            const T *const freeB0       = unknowns[8];
            const T *const freeB1       = unknowns[9];
            //const T *const collTime     = unknowns[10];
            const T *const momentumA0   = unknowns[11];
            const T *const momentumA1   = unknowns[12];
            const T *const momentumB0   = unknowns[13];
            const T *const momentumB1   = unknowns[14];

            T g[3];
            GravityCostFunctor::getWorldGravity( rotG, a, _fps, g);

            T vA0[3], vA1[3], vB0[3], vB1[3];
            LinVelCostFunctor::getLinearVelocity(rotG, freeA0, T(_startFrameId), T(_dt), T(_fps), g, vA0);
            //std::cout << "vA0: " << printJetVector3( vA0 ) << " * " << massA[0] << std::endl;
            LinVelCostFunctor::getLinearVelocity(rotG, freeA1, T(_startFrameId), T(_dt), T(_fps), g, vA1);
            //std::cout << "vA1: " << printJetVector3( vA1 ) << " * " << massA[0] << std::endl;
            LinVelCostFunctor::getLinearVelocity(rotG, freeB0, T(_startFrameId), T(_dt), T(_fps), g, vB0);
            //std::cout << "vB0: " << printJetVector3( vB0 ) << " * " << massB[0] <<  std::endl;
            LinVelCostFunctor::getLinearVelocity(rotG, freeB1, T(_startFrameId), T(_dt), T(_fps), g, vB1);
            //std::cout << "vB1: " << printJetVector3( vB1 ) << " * " << massB[0] <<  std::endl;

            // todo: depr, these are equivalent to translations
            T cA0[3], cA1[3], cB0[3], cB1[3];
            ParabolaCostFunctor::getPositionAtTime(cA0, rotG, a, translationA, freeA0, T(0.) );
            ParabolaCostFunctor::getPositionAtTime(cA1, rotG, a, translationA, freeA1, T(0.) );
            ParabolaCostFunctor::getPositionAtTime(cB0, rotG, a, translationB, freeB0, T(0.) );
            ParabolaCostFunctor::getPositionAtTime(cB1, rotG, a, translationB, freeB1, T(0.) );

            T linMomA0[3], linMomA1[3], linMomB0[3], linMomB1[3];
            ConservationOfLinearMomentum::getLinMomentum(massA, vA0, linMomA0);
            ConservationOfLinearMomentum::getLinMomentum(massA, vA1, linMomA1);
            ConservationOfLinearMomentum::getLinMomentum(massB, vB0, linMomB0);
            ConservationOfLinearMomentum::getLinMomentum(massB, vB1, linMomB1);

            T tmp[3];

            T before[3];
            ceres::CrossProduct(cA0, linMomA0, tmp);
            ceres::equal3(tmp, before);
            ceres::CrossProduct(cB0, linMomB0, tmp);
            ceres::plusEqual3(tmp, before);
            ceres::plusEqual3(momentumA0, before);
            ceres::plusEqual3(momentumB0, before);

            T after[3];
            ceres::CrossProduct(cA1, linMomA1, tmp);
            ceres::equal3(tmp, after);
            ceres::CrossProduct(cB1, linMomB1, tmp);
            ceres::plusEqual3(tmp, after);
            ceres::plusEqual3(momentumA1, after);
            ceres::plusEqual3(momentumB1, after);

            residuals[0] = T(_sqrtWeight) * (before[0] - after[0]); // todo: normalize terms
            residuals[1] = T(_sqrtWeight) * (before[1] - after[1]);
            residuals[2] = T(_sqrtWeight) * (before[2] - after[2]);

            return true;
        } //...operator()

        // Factory to hide the construction of the CostFunction object from the client code.
        static ceres::CostFunction *Create(const CeresScalar sqrtWeight, const CeresScalar fps, const CeresScalar dt, const CeresScalar startFrameId)
        {
            auto fn = new ceres::DynamicAutoDiffCostFunction<ConservationOfAngularMomentum>(new ConservationOfAngularMomentum(sqrtWeight, fps, dt, startFrameId));
            fn->AddParameterBlock(PhysIndexer::MASS_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MASS_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_A_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_TRANSLATION_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_TRANSLATION_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
            fn->AddParameterBlock(PhysIndexer::COLL_TIME_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
            fn->SetNumResiduals(NUM_RESIDUALS);
            return fn;
        } //...Create()

    protected:
        const CeresScalar _sqrtWeight;       //!< Sqrt of term weight.
        const CeresScalar _dt;               //!< Timestep per frame.
        const CeresScalar _fps;              //!< Recording speed.
        const CeresScalar _startFrameId;     //!< At what time the pose equality has to hold.

}; //...ConservationOfAngularMomentum

struct ConservationOfLinearMomentumInfMass {
        using CeresScalar = ceres::CeresScalar;

        enum { NUM_RESIDUALS = 3 };

        /**
         *  \param[in] sqrtWeight   Sqrt of term weight.
         *  \param[in] dt           Timestep size.
         *  \param[in] fps          Recording speed (\f$ \frac{1}{dt} \f$).
         *  \param[in] startFrameId Anchor time point for all parabolas.
         */
        ConservationOfLinearMomentumInfMass(const CeresScalar sqrtWeight, const CeresScalar fps, const CeresScalar dt, const CeresScalar startFrameId)
            : _sqrtWeight(sqrtWeight), _dt(dt), _fps(fps), _startFrameId(startFrameId) { }

        /** \brief Enforces \f$ \mathbf{v}_a M_a + \mathbf{v}_b M_b = \mathbf{v}'_a M_a + \mathbf{v}'_b M_b \f$ at the time of collision \p collTime.
         * \param[in] massA Mass of first object.
         * \param[in] massB Mass of second object.
         * \param[in] shared Shared parabola parameters (Two rotation angles \f$ \theta_X \f$ and \f$ \theta_Y \f$ and squared coefficient \f$ a \f$).
         * \param[in] freeA0 Parabola parameters of first object before collision.
         * \param[in] freeA1 Parabola parameters of first object after collision.
         * \param[in] freeB0 Parabola parameters of second object before collision.
         * \param[in] freeB1 Parabola parameters of second object after collision.
         * \param[in] collTime Exact, floating point time point of collision.
         */
        template<typename T>
        bool operator()(const T massA[1], const T rotG[2], const  T a[1], const T *freeA0, const T *freeA1/*, const T collTime[1]*/, T *residuals) const
        {
            using ceres::printJet;
            using ceres::printJetVector3;

            T g[3];
            GravityCostFunctor::getWorldGravity(rotG, a, _fps, g);

            T vA0[3], vA1[3];
            LinVelCostFunctor::getLinearVelocity(rotG, freeA0, T(_startFrameId), T(_dt), T(_fps), g, vA0);
            LinVelCostFunctor::getLinearVelocity(rotG, freeA1, T(_startFrameId), T(_dt), T(_fps), g, vA1);

            T linMomA0[3], linMomA1[3];
            ConservationOfLinearMomentum::getLinMomentum(massA, vA0, linMomA0);
            ConservationOfLinearMomentum::getLinMomentum(massA, vA1, linMomA1);
            std::cout << ceres::printJetVector3( linMomA0 ) << " vs. " << ceres::printJetVector3( linMomA1 ) << std::endl;

            residuals[0] = T(_sqrtWeight) * (linMomA1[0] + linMomA0[0]);
            residuals[1] = T(_sqrtWeight) * (linMomA1[1] + linMomA0[1]);
            residuals[2] = T(_sqrtWeight) * (linMomA1[2] + linMomA0[2]);

            return true;
        } //...operator()

        // Factory to hide the construction of the CostFunction object from the client code.
        static ceres::CostFunction *Create(const CeresScalar sqrtWeight, const CeresScalar fps, const CeresScalar dt, const CeresScalar startFrameId)
        {
            return (new ceres::AutoDiffCostFunction<ConservationOfLinearMomentumInfMass, NUM_RESIDUALS,
                                                    PhysIndexer::MASS_STRIDE, PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE, PhysIndexer::PARABOLA_SHARED_A_STRIDE,
                                                    PhysIndexer::PARABOLA_FREE_STRIDE, PhysIndexer::PARABOLA_FREE_STRIDE/*,PhysIndexer::COLL_TIME_STRIDE*/>(
                new ConservationOfLinearMomentumInfMass(sqrtWeight, fps, dt, startFrameId)));
        }

    protected:
        const CeresScalar _sqrtWeight;       //!< Sqrt of term weight.
        const CeresScalar _dt;               //!< Timestep per frame.
        const CeresScalar _fps;              //!< Recording speed.
        const CeresScalar _startFrameId;     //!< Time, when the parabola transformations are applied.

}; //...struct ConservationOfLinearMomentumInfMass

/** \brief Enforces \f$ \mathbf{c}_a \times (\mathbf{v}_a M_a)  + \mathbf{c}_b \times (\mathbf{v}_b M_b) + \mathbf{L}_a + \mathbf{L}_b =
                        \mathbf{c}_a \times (\mathbf{v}'_a M_a) + \mathbf{c}_b \times (\mathbf{v}'_b M_b) + \mathbf{L}'_a + \mathbf{L}'_b \f$ at the time of collision. */
struct ConservationOfAngularMomentumInfMass {
        using CeresScalar = ceres::CeresScalar;

        enum { NUM_RESIDUALS = 3 };

        /**
         *  \param[in] sqrtWeight   Sqrt of term weight.
         *  \param[in] dt           Timestep size.
         *  \param[in] fps          Recording speed (\f$ \frac{1}{dt} \f$).
         *  \param[in] startFrameId Anchor time point for all parabolas.
         */
        ConservationOfAngularMomentumInfMass(const CeresScalar sqrtWeight, const CeresScalar fps, const CeresScalar dt, const CeresScalar startFrameId)
            : _sqrtWeight(sqrtWeight), _dt(dt), _fps(fps), _startFrameId(startFrameId) { }

        template<typename T>
        bool operator()(
            const T *const *unknowns, T *residuals) const
        {
            using ceres::printJet;
            using ceres::printJetVector3;

            const T *const massA        = unknowns[0];
            const T *const rotG         = unknowns[1];
            const T *const a            = unknowns[2];

            const T *const translationA = unknowns[3];
            const T *const freeA0       = unknowns[4];
            const T *const freeA1       = unknowns[5];
            //const T* const translationB = unknowns[6];
            //const T* const freeB0 = unknowns[7];
            //const T* const freeB1 = unknowns[8];
            //const T *const collTime     = unknowns[6];
            const T *const momentumA0   = unknowns[7];
            const T *const momentumA1   = unknowns[8];
            //const T* const momentumB0 = unknowns[12];
            //const T* const momentumB1 = unknowns[13];

            T g[3];
            GravityCostFunctor::getWorldGravity(rotG, a, _fps, g);

            T vA0[3], vA1[3];
            LinVelCostFunctor::getLinearVelocity(rotG, freeA0, T(_startFrameId), T(_dt), T(_fps), g, vA0);
            LinVelCostFunctor::getLinearVelocity(rotG, freeA1, T(_startFrameId), T(_dt), T(_fps), g, vA1);

            T cA0[3], cA1[3];
            ParabolaCostFunctor::getPositionAtTime(cA0, rotG, a, translationA, freeA0, T(0.) /*collTime*/ );
            ParabolaCostFunctor::getPositionAtTime(cA1, rotG, a, translationA, freeA1, T(0.) /*collTime*/ );

            T linMomA0[3], linMomA1[3];
            ConservationOfLinearMomentum::getLinMomentum(massA, vA0, linMomA0);
            ConservationOfLinearMomentum::getLinMomentum(massA, vA1, linMomA1);

            T tmp[3];

            T before[3];
            ceres::CrossProduct(cA0, linMomA0, tmp);
            ceres::equal3(tmp, before);
            ceres::plusEqual3(momentumA0, before);

            T after[3];
            ceres::CrossProduct(cA1, linMomA1, tmp);
            ceres::equal3(tmp, after);
            ceres::plusEqual3(momentumA1, after);

            residuals[0] = T(_sqrtWeight) * (after[0] + before[0]);
            residuals[1] = T(_sqrtWeight) * (after[1] + before[1]);
            residuals[2] = T(_sqrtWeight) * (after[2] + before[2]);
            std::cout << "angMomBefore: " << printJet(before[0]) << "," << printJet(before[1]) << "," << printJet(before[2]) << std::endl;
            std::cout << "angMomAfter: "  << printJet(after[0])  << "," << printJet(after[1])  << "," << printJet(after[2])  << std::endl;

            return true;
        } //...operator()

        // Factory to hide the construction of the CostFunction object from the client code.
        static ceres::CostFunction *Create(const CeresScalar sqrtWeight, const CeresScalar fps, const CeresScalar dt, const CeresScalar startFrameId)
        {
            auto fn = new ceres::DynamicAutoDiffCostFunction<ConservationOfAngularMomentumInfMass>(new ConservationOfAngularMomentumInfMass(sqrtWeight, fps, dt, startFrameId));
            fn->AddParameterBlock(PhysIndexer::MASS_STRIDE);
            //fn->AddParameterBlock( PhysIndexer::MASS_STRIDE );
            fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_SHARED_A_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_TRANSLATION_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
            fn->AddParameterBlock(PhysIndexer::PARABOLA_FREE_STRIDE);
            //fn->AddParameterBlock( PhysIndexer::PARABOLA_TRANSLATION_STRIDE );
            //fn->AddParameterBlock( PhysIndexer::PARABOLA_FREE_STRIDE );
            //fn->AddParameterBlock( PhysIndexer::PARABOLA_FREE_STRIDE );
            fn->AddParameterBlock(PhysIndexer::COLL_TIME_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
            fn->AddParameterBlock(PhysIndexer::MOMENTUM_STRIDE);
            //fn->AddParameterBlock( PhysIndexer::MOMENTUM_STRIDE );
            //fn->AddParameterBlock( PhysIndexer::MOMENTUM_STRIDE );
            fn->SetNumResiduals(NUM_RESIDUALS);
            return fn;
        } //...Create()

    protected:
        const CeresScalar _sqrtWeight;       //!< Sqrt of term weight.
        const CeresScalar _dt;               //!< Timestep per frame.
        const CeresScalar _fps;              //!< Recording speed.
        const CeresScalar _startFrameId;     //!< At what time the pose equality has to hold.

}; //...struct ConservationOfAngularMomentum
} //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_CONSERVATIONCOSTFUNCTORS_H
