//
// Created by bontius on 03/04/16.
//

#ifndef TRACKVIDEO_PHYS_IMPULSECOSTFUNCTIONS_H
#define TRACKVIDEO_PHYS_IMPULSECOSTFUNCTIONS_H

#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"
#include "tracking/phys/physIndexer.h"
#include "tracking/common/typedefs.h"
//#include "tracking/common/util/energyPlotter.h"
#include "tracking/common/util/exception.h"
#include "ceres/ceresUtil.h"
#include "ceres/typedefs.h"
#include "ceres/cost_function.h"

namespace tracking {
namespace bundle_physics {

DEFINE_EXCEPTION(LinVelCostFunctor_ParticipantIdLargerThan1)

/** \brief Impulse and linear velocity equations. */
struct LinVelCostFunctor {
    public:
        enum { NUM_RESIDUALS = 3 };
        using CeresScalar = ceres::CeresScalar;
    public:
        LinVelCostFunctor(CeresScalar const sqrtWeight, CeresScalar const fps, FrameId const startFrameId,
                          int const participantId)
            : _sqrtWeight(sqrtWeight),
              _fps(fps),
              _dt(CeresScalar(1.) / fps),
              _startFrame(startFrameId),
              _participantId(participantId)
        {
            if (_participantId > 1)
                throw new LinVelCostFunctor_ParticipantIdLargerThan1Exception("0 or 1");
        }

        template<typename T>
        static void getStartingVelocity(T const rotG[2], const T *const free, const T fps, T *v0) {
            // v0( s[0] * fps, b[0] * fps           , 0. ); // starting velocity
            T v0_local[3];
            v0_local[0] = free[PhysIndexer::PARABOLA_FREE_S_OFFSET] * fps;
            v0_local[1] = free[PhysIndexer::PARABOLA_FREE_B_OFFSET] * fps;
            v0_local[2] = T(0.);
            rotatePoint(free + PhysIndexer::PARABOLA_FREE_ANGLE_OFFSET, rotG, v0_local, v0);
        }

        template<typename T, typename T2>
        static void getLinearVelocity(T const rotG[2], const T *const free, const T2 startFrame, const T2 dt, const T2 fps, const T2 g[3], T v[3]) {
            T v0[3];
            getStartingVelocity(rotG, free, fps, v0);
            T const fullDt = T(0. - startFrame) * T(dt);

            v[0] = v0[0] + g[0] * fullDt;
            v[1] = v0[1] + g[1] * fullDt;
            v[2] = v0[2] + g[2] * fullDt;
        } //...getLinearVelocity()

        template<typename T, typename T2>
        static void getLinearVelocityAtTime(T const rotG[2], T const *const free, T const *time, const T2 startFrame, const T2 dt, const T2 fps, const T2 g[3], T v[3])
        {
            T v0[3];
            getStartingVelocity(rotG, free, fps, v0);
            const T fullDt = (time[0] - T(startFrame)) * T(dt);

            v[0] = v0[0] + g[0] * fullDt;
            v[1] = v0[1] + g[1] * fullDt;
            v[2] = v0[2] + g[2] * fullDt;
        } //...getLinearVelocity()

        // va' = va + Jn/1
        // vb' = vb - Jn/(Mb/Ma)
        template<typename T>
        bool operator()(T const rotG[2],
                        T const a[1],
                        T const freeA0[3],
                        T const freeA1[3],
                        T const impulse[3],
                        T const mass[1],
                        T* residuals) const
        {
            using ceres::getVal;

            // gravity
            T g[3];
            GravityCostFunctor::getWorldGravity(rotG, a, _fps, g);

            T vA0[3], vA1[3];
            getLinearVelocity(rotG, freeA0, T(_startFrame), T(_dt), T(_fps), g, vA0);
            getLinearVelocity(rotG, freeA1, T(_startFrame), T(_dt), T(_fps), g, vA1);

            #if 0
            if ( _participantId )
                {
                    residuals[0] = T(_sqrtWeight) * (vA0[0] - m[0] / mass[0] - vA1[0]);
                    residuals[1] = T(_sqrtWeight) * (vA0[1] - m[1] / mass[0] - vA1[1]);
                    residuals[2] = T(_sqrtWeight) * (vA0[2] - m[2] / mass[0] - vA1[2]);
                }
                else
                {
                    residuals[0] = T(_sqrtWeight) * (vA0[0] + m[0] / mass[0] - vA1[0]);
                    residuals[1] = T(_sqrtWeight) * (vA0[1] + m[1] / mass[0] - vA1[1]);
                    residuals[2] = T(_sqrtWeight) * (vA0[2] + m[2] / mass[0] - vA1[2]);
                }
            #else
            if (_participantId) {
                residuals[0] = T(_sqrtWeight) * ((vA0[0] - vA1[0]) * mass[0] - impulse[0]);
                residuals[1] = T(_sqrtWeight) * ((vA0[1] - vA1[1]) * mass[0] - impulse[1]);
                residuals[2] = T(_sqrtWeight) * ((vA0[2] - vA1[2]) * mass[0] - impulse[2]);
            } else {
                residuals[0] = T(_sqrtWeight) * ((vA0[0] - vA1[0]) * mass[0] + impulse[0]);
                residuals[1] = T(_sqrtWeight) * ((vA0[1] - vA1[1]) * mass[0] + impulse[1]);
                residuals[2] = T(_sqrtWeight) * ((vA0[2] - vA1[2]) * mass[0] + impulse[2]);
            }
            #endif

            #if 0
            T    resLen = ceres::length(residuals[0], residuals[1], residuals[2]);
            char name[255];
            sprintf(name, "linVelResidual obj%d", _participantId);
            EnergyPlotter::getInstance().addValue(*getVal(&resLen), "linVelResidual (Eq3)", name);
            #endif

            return true;
        } //...operator()

        static ceres::CostFunction *Create(const ceres::CeresScalar sqrtWeight, const ceres::CeresScalar fps, const FrameId startFrameId, const int participantId)
        {
            return new ceres::AutoDiffCostFunction<LinVelCostFunctor,
                                                   NUM_RESIDUALS,
                                                   PhysIndexer::PARABOLA_SHARED_ANGLES_STRIDE, PhysIndexer::PARABOLA_SHARED_A_STRIDE,
                                                   PhysIndexer::PARABOLA_FREE_STRIDE, PhysIndexer::PARABOLA_FREE_STRIDE, /*PhysIndexer::COLL_TIME_STRIDE, */PhysIndexer::IMPULSE_STRIDE, PhysIndexer::MASS_STRIDE>
                (new LinVelCostFunctor(sqrtWeight, fps, startFrameId, participantId));
        }

    protected:
        const CeresScalar _sqrtWeight;
        const CeresScalar _fps;
        const CeresScalar _dt;
        const FrameId     _startFrame;
        const int         _participantId;
}; //...struct VelocityCostFunctor

#define VIS 0

/** \brief \f$ L_a' = L_a + ( x_a x Jn / 1 ), L_b' = L_b - ( x_b x Jn / (Mb/Ma) ) \f$ */
struct AngVelCostFunctor {
        DEFINE_EXCEPTION(AngVelCostFunctor_ParticipantIdLargerThan1)

        enum { NUM_RESIDUALS = 3 };
        typedef ceres::CeresScalar CeresScalar;
    public:
        template<typename _Vector3>
        AngVelCostFunctor(const ceres::CeresScalar sqrtWeight, const _Vector3 &size, const int participantId)
            : _sqrtWeight(sqrtWeight), _sqrSize({size(0) * size(0), size(1) * size(1), size(2) * size(2)}), _participantId(participantId)
        {
            if (_participantId > 1)
                throw new AngVelCostFunctor_ParticipantIdLargerThan1Exception("0 or 1");
        }

        // L_a' = L_a + ( x_a x Jn / 1       )
        // L_b' = L_b - ( x_b x Jn / (Mb/Ma) )
        template<typename T>
        bool operator()(T const momentum0[3],
                        T const momentum1[3],
                        T const impulse[3],
                        T const collPoint[3],
                        T const translationThis[3],
                        T const translationOther[3],
                        T* residuals) const {
#if VIS
            typedef float Scalar;
                static Soup::vis::Visualizer<Scalar> vis("angVelfunctor");
                vis.removeActorsAll();
#endif
            using ceres::printJet;
            using ceres::CeresVector3;
            using ceres::MapConstCeresVector3;
            using ceres::getVal;
            using ceres::printJetVector3;

            residuals[0] = momentum0[0] - momentum1[0];
            residuals[1] = momentum0[1] - momentum1[1];
            residuals[2] = momentum0[2] - momentum1[2];
#if VIS
            CeresVector3 tM0( *getVal(momentum0), *getVal(momentum0+1), *getVal(momentum0+2) );
                CeresVector3 tM1( *getVal(momentum1), *getVal(momentum1+1), *getVal(momentum1+2) );
                CeresVector3 tImpulse( *getVal(impulse), *getVal(impulse+1), *getVal(impulse+2) );
#endif

            T finalTorque[3];
            if (_participantId) {
                // this == B
                // other == A
                // x_b = c_a + x_a - c_b = x_a + c_a - c_b = x_a + (other - this)
                T baPlusCollPoint[3];
                baPlusCollPoint[0] = translationOther[0] - translationThis[0] + collPoint[0];
                baPlusCollPoint[1] = translationOther[1] - translationThis[1] + collPoint[1];
                baPlusCollPoint[2] = translationOther[2] - translationThis[2] + collPoint[2];

                // m2 (cA1-cB1+x1) - m1 (cA2-cB2+x2),
                // m0 (cA2-cB2+x2) - m2 (cA0-cB0+x0),
                // m1 (cA0-cB0+x0) - m0 (cA1-cB1+x1)

                // - ( m_z x(b)_y - m_y x(b)_z )
                finalTorque[0] = impulse[2] * baPlusCollPoint[1] - impulse[1] * baPlusCollPoint[2];
                residuals[0] -= finalTorque[0];

                // - ( m_x x(b)_z - m_z x(b)_x )
                finalTorque[1] = impulse[0] * baPlusCollPoint[2] - impulse[2] * baPlusCollPoint[0];
                residuals[1] -= finalTorque[1];

                // - ( m_y x(b)_x - m_x x(b)_y )
                finalTorque[2] = impulse[1] * baPlusCollPoint[0] - impulse[0] * baPlusCollPoint[1];
                residuals[2] -= finalTorque[2];
#if VIS
                CeresVector3 tB( *getVal(translationThis), *getVal(translationThis+1), *getVal(translationThis+2) );
                    vis.addSphere( tB.cast<Scalar>(), 0.01, Eigen::Vector3f(1.,0.,0.), "thisSphere" );
                    CeresVector3 tA( *getVal(translationOther), *getVal(translationOther+1), *getVal(translationOther+2) );
                    vis.addSphere( tA.cast<Scalar>(), 0.01, Eigen::Vector3f(0.,1.,0.), "otherSphere" );

                    CeresVector3 cPB( *getVal(baPlusCollPoint), *getVal(baPlusCollPoint+1), *getVal(baPlusCollPoint+2) );
                    vis.addLine( tB.cast<Scalar>(), tB.cast<Scalar>() + cPB.cast<Scalar>(), Eigen::Vector3f(0.,0.,0.), "ba" );vis.addArrow( (tB+cPB).cast<Scalar>(), (tB+cPB).cast<Scalar>() + tImpulse.cast<Scalar>(), Eigen::Vector3f(0.,0.,0.), "cImpulse" );
                    vis.addArrow( tB, tB+tM0, Eigen::Vector3f(1.,0.,0.), "m0");
                    vis.addArrow( tB, tB+tM1, Eigen::Vector3f(0.,1.,0.), "m1");
#endif
            } else {
                // m_z x(a)_y - m_y x(a)_z
                finalTorque[0] = impulse[2] * collPoint[1] - impulse[1] * collPoint[2];
                residuals[0] += finalTorque[0];

                // m_x x(a)_z - m_z x(a)_x
                finalTorque[1] = impulse[0] * collPoint[2] - impulse[2] * collPoint[0];
                residuals[1] += finalTorque[1];

                // m_y x(a)_x - m_x x(a)_y
                finalTorque[2] = impulse[1] * collPoint[0] - impulse[0] * collPoint[1];
                residuals[2] += finalTorque[2];
#if VIS
                CeresVector3 tA( *getVal(translationThis), *getVal(translationThis+1), *getVal(translationThis+2) );
                    vis.addSphere( tA.cast<Scalar>(), 0.01, Eigen::Vector3f(1.,0.,0.), "thisSphere" );
                    CeresVector3 tB( *getVal(translationOther), *getVal(translationOther+1), *getVal(translationOther+2) );
                    vis.addSphere( tB.cast<Scalar>(), 0.01, Eigen::Vector3f(0.,1.,0.), "otherSphere" );

                    CeresVector3 tCp( *getVal(collPoint), *getVal(collPoint+1), *getVal(collPoint+2) );
                    vis.addLine( tA.cast<Scalar>(), tA.cast<Scalar>() + tCp.cast<Scalar>(), Eigen::Vector3f(0.,0.,0.), "cP" );
                    vis.addArrow( (tA+tCp).cast<Scalar>(), (tA+tCp).cast<Scalar>() + tImpulse.cast<Scalar>(), Eigen::Vector3f(0.,0.,0.), "cImpulse" );
                    vis.addArrow( tA, tA+tM0, Eigen::Vector3f(1.,0.,0.), "m0" );
                    vis.addArrow( tA, tA+tM1, Eigen::Vector3f(0.,1.,0.), "m1" );
#endif
            }
#if VIS
            vis.addCoordinateSystem(0.1);
                vis.spinOnce(10);
#endif
            residuals[0] *= T(_sqrtWeight);
            residuals[1] *= T(_sqrtWeight);
            residuals[2] *= T(_sqrtWeight);
#if 0
            T    resLen = ceres::length(residuals[0], residuals[1], residuals[2]);
            char name[255];
            sprintf(name, "angVelResidual obj%d (Eq4)", _participantId);
            EnergyPlotter::getInstance().addValue(*getVal(&resLen), name);
            EnergyPlotter::getInstance().addValue(*getVal(finalTorque), name, "torque (x x Jn)[0]");
            EnergyPlotter::getInstance().addValue(*getVal(finalTorque + 1), name, "torque (x x Jn)[1]");
            EnergyPlotter::getInstance().addValue(*getVal(finalTorque + 2), name, "torque (x x Jn)[2]");

            T impulseLen = ceres::length(impulse[0], impulse[1], impulse[2]);
            EnergyPlotter::getInstance().addValue(*getVal(&impulseLen), name, "|Jn|");

            if (_participantId)
            {
                sprintf(name, "Jn");
                EnergyPlotter::getInstance().addValue(*getVal(impulse), name, "Jn[0]");
                EnergyPlotter::getInstance().addValue(*getVal(impulse + 1), name, "Jn[1]");
                EnergyPlotter::getInstance().addValue(*getVal(impulse + 2), name, "Jn[2]");
            }

            sprintf(name, "L obj%d", _participantId);
            EnergyPlotter::getInstance().addValue(*getVal(momentum0), name, "L[0]");
            EnergyPlotter::getInstance().addValue(*getVal(momentum0 + 1), name, "L[1]");
            EnergyPlotter::getInstance().addValue(*getVal(momentum0 + 2), name, "L[2]");
            EnergyPlotter::getInstance().addValue(*getVal(momentum1), name, "L_after[0]");
            EnergyPlotter::getInstance().addValue(*getVal(momentum1 + 1), name, "L_after[1]");
            EnergyPlotter::getInstance().addValue(*getVal(momentum1 + 2), name, "L_after[2]");
#endif
            return true;
        } //...operator()

        template<typename _Vector3>
        static ceres::CostFunction *Create(const ceres::CeresScalar sqrtWeight, const _Vector3 &size, const int participantId)
        {
            return new ceres::AutoDiffCostFunction<AngVelCostFunctor,
                                                   NUM_RESIDUALS,
                                                   PhysIndexer::MOMENTUM_STRIDE, PhysIndexer::MOMENTUM_STRIDE, PhysIndexer::IMPULSE_STRIDE, PhysIndexer::COLL_POINTS_STRIDE,
                                                   PhysIndexer::PARABOLA_TRANSLATION_STRIDE, PhysIndexer::PARABOLA_TRANSLATION_STRIDE/*, PhysIndexer::COLL_TIME_STRIDE*/>
                (new AngVelCostFunctor(sqrtWeight, size, participantId));
        }

    protected:
        const CeresScalar                _sqrtWeight;
        const std::array<CeresScalar, 3> _sqrSize;
        const int                        _participantId;
}; //...struct AngVelCostFunctor

} //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_IMPULSECOSTFUNCTIONS_H
