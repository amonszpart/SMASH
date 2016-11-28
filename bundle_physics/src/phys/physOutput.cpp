#include "tracking/phys/energyTerms/impl/poseTerm.hpp"
#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/energyTerms/impl/impulseTerm.hpp"
#include "tracking/phys/physOutput.h"
#include "tracking/phys/initialize/assignments.h"
#include "tracking/phys/bundleWithPhysics.h"
#include "tracking/phys/partUtil.h"
#include "tracking/phys/ceresUtil.h"
#include "tracking/common/util/energyPlotter.h"
#include "tracking/vis/visualizer.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/util/util.h"

namespace tracking {
namespace bundle_physics {

struct Condition
{
    virtual bool operator()(const FrameId a, const FrameId b) const = 0;
};

struct LessEqual : public Condition
{
    virtual bool operator()(const FrameId a, const FrameId b) const override { return a <= b; }
};

struct GreaterEqual : public Condition
{
    virtual bool operator()(const FrameId a, const FrameId b) const override { return a >= b; }
};

template<typename _Scalar>
inline Eigen::Matrix<_Scalar, 3, 1> getVelAtCollPoint(const Eigen::Matrix<_Scalar, 4, 4> &/*T*/, const Eigen::Matrix<_Scalar, 3, 1> &relCollPoint, const Eigen::Matrix<_Scalar, 3, 1> &omega)
{
    //const auto xAobjA = T.template block<3,3>(0,0).inverse() * relCollPoint;
    //return T.template block<3,3>(0,0) * omega.cross(xAobjA);
    std::cout << "getVelAtcollpoint: " << omega.transpose() << std::endl;
    return omega.cross(relCollPoint);
}

template<typename T>
int sgn(T val)
{
    return (T(0) < val) - (val < T(0));
}

void output(BundleWithPhysicsResult &out, const CuboidsT &cuboids, const GroupedTracks2d &tracks2d, const FrameIdsT &frameIds, const PhysIndexer &indexer, const Consts &consts, const bool doVis, TracksToCuboidsT const*const assignments) {
    using ceres::CeresScalar;
    using ceres::CeresVector3;
    using ceres::CeresVector4;
    using ceres::MapCeresVector3;
    using ceres::MapConstCeresVector3;
    using ceres::MapCeresVector4;
    using ceres::MapConstCeresVector4;
    using ceres::CeresMatrix3;
    //typedef Cuboid::Scalar      Scalar;
    //typedef Cuboid::QuaternionT QuaternionT;
    //typedef Cuboid::Vector3     Vector3;

    const CeresScalar subDt = consts.k_dt / consts.substeps;

    if (!out.cuboids) {
        out.cuboids.reset(new CuboidsT());
    }
    else { out.cuboids->clear(); }
    out.clear();

    // gravity
    //const CeresScalar *const parabolaShared(indexer.getParabolaSharedParamsConst());
    const CeresScalar *const parabolaRotShared(indexer.getParabolaRotationSharedConst());
    const CeresScalar *const parabolaA(indexer.getParabolaSquaredParamConst());
    const CeresVector3 g =
                           [&indexer, &parabolaRotShared, &parabolaA, &consts]() -> CeresVector3
                           {
                               CeresMatrix3 R;
                               CeresScalar  y0(0.);
                               toRotationMatrix(&y0, parabolaRotShared, R);
                               return R * (CeresVector3() << 0., 2. * parabolaA[0] * consts.k_fps * consts.k_fps, 0.).finished();
                           }();
    {
        out.a       = *parabolaA;
        out.rotX    = parabolaRotShared[0];
        out.rotY1   = parabolaRotShared[1];
        out.gravity = g.cast<Scalar>();

        // TODO: replace above with below
        CeresVector3 g2;
        GravityCostFunctor::getWorldGravity(parabolaRotShared, parabolaA, consts.k_fps, g2.data() );
        if ((g - g2).norm() > kSmallDiff)
        {
            std::cerr << "g: " << g.transpose() << ", g2: " << g2.transpose() << std::endl;
            //throw new BundleWithPhysics_GravityDiscrepancyException("");
        }
    }

    for (CuboidId cuboidId = 0; cuboidId != static_cast<CuboidId>(cuboids.size()); ++cuboidId) {
        out.cuboids->push_back(cuboids.at(cuboidId));
        if (static_cast<CuboidId>(out.cuboids->size()) != cuboidId + 1)
            throw new BundleWithPhysics_OutputDiscrepancyException("");
        Cuboid &outCuboid = out.cuboids->at(cuboidId);
        outCuboid.clearStates();

        if ( !cuboids.at(cuboidId).isMassFinite() ) {
            Cuboid const& cuboid = cuboids.at(cuboidId);
            outCuboid.setMass( cuboid.getMass() );
            for (FrameId frameId = frameIds.front(); frameId!=frameIds.back(); step(frameId) ) {
                if ( outCuboid.getStates().size() ) {
                    auto state = cuboid.getStates().begin()->second;
                    outCuboid.setPose(frameId, state.getPose());
                    outCuboid.setPosition(frameId, state.getPosition() );
                } else {
                    outCuboid.setPosition(frameId, Vector3(0., 0., 0.));
                    outCuboid.setPose(frameId, QuaternionT::Unit());
                }
                //outCuboid.setPose( frameId, cuboid.getStates().begin(). );
                outCuboid.setLinVel( frameId, Vector3::Zero() );
                outCuboid.setAngVel( frameId, Vector3::Zero() );
            }

            for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
                out.collisions[collId].states.first[cuboidId] = outCuboid.getState(frameIds.front());
                out.collisions[collId].states.second[cuboidId] = outCuboid.getState(frameIds.front());
            }
            continue;
        }

        // set mass
        const CeresScalar *const mass = indexer.getMassConst(cuboidId);
        outCuboid.setMass(mass[0]);

        CeresVector3 invI;
        outCuboid.getInverseIFromMass( invI, mass[0] );
        if ( outCuboid.getShape() != cuboids.at(cuboidId).getShape() )
            throw new BundleWithPhysics_Output_ShapeMismatchException("");

        for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
            const CeresScalar *const collTime    = indexer.getCollisionTimeConst(collId);
            const CeresScalar *const collImpulse = indexer.getImpulseConst(collId);
            const CeresScalar *const collPoint   = indexer.getCollisionPointConst(collId);

            out.collisions[collId].impulse      = MapConstCeresVector3(collImpulse).cast<Scalar>();
            out.collisions[collId].relCollPoint = MapConstCeresVector3(collPoint).cast<Scalar>();
            out.collTimes[collId] = collTime[0];

            for (PartId partId = collId; partId <= PartId(collId) + 1; ++partId) {
                // only forwards, except for first collision
                if (collId && (collId == partId))
                    continue;

                // estimate bounds
                FrameId                    partStart  = getPartStart(partId, frameIds);
                FrameId                    partEnd    = getPartEnd(partId, frameIds);
                auto                       stepFn     = &step;
                int                        subStepDir = 1;
                std::unique_ptr<Condition> stopCondition(new LessEqual());

                // reverse, if very first part to propagate backwards from collision pose
                if (partId == consts.firstPartId) {
                    std::swap(partStart, partEnd);
                    stepFn     = &backStep;
                    subStepDir = -1;
                    stopCondition.reset(new GreaterEqual());
                }

                const CeresScalar *const parabolaTranslation(indexer.getParabolaTranslationConst(cuboidId, collId));
                const CeresScalar *const parabolaFree(indexer.getParabolaFreeParamsConst(cuboidId, partId));
                const CeresScalar *const parabolaRotFree(indexer.getParabolaRotationFreeConst(cuboidId, partId));
                const CeresScalar *const b(indexer.getParabolaBParamConst(cuboidId, partId));
                const CeresScalar *const s(indexer.getParabolaSParamConst(cuboidId, partId));
                MapConstCeresVector3 momentum(indexer.getMomentumConst(cuboidId, partId));
                std::cout << "momentum[" << cuboidId << "][" << partId << "]: " << momentum.transpose() << std::endl;

                // TODO: change to getInitialVelocity
                CeresMatrix3 R;
                toRotationMatrix(parabolaRotFree, parabolaRotShared, R);
                CeresVector3 v0(s[0] * consts.k_fps, b[0] * consts.k_fps, 0.); // starting velocity
                v0 = R * v0;

                if (consts.substeps % 2 != 1)
                    throw new BundleWithPhysics_AnimateCuboids_SubstepsHaveToBeOddException("");

                // save parabola
                {
                    Parabola parabola;
                    parabola.setTranslation(parabolaTranslation);
                    parabola.b     = *b;
                    parabola.s     = *s;
                    parabola.rotY0 = parabolaRotFree[0];
                    if (out.hasParabola(cuboidId, partId))
                        throw new BundleWithPhysics_OutputParabolaAlreadyExistsException("");
                    out.parabolas[cuboidId][partId] = parabola;
                }
                // save dynamics
                {
                    if (out.hasMomentum(cuboidId, partId))
                        throw new BundleWithPhysics_OutputMomentumAlreadyExistsException("");
                    out.momenta[cuboidId][partId] = momentum.cast<Scalar>();
                }

                bool         atCollision(true);
                CeresVector4 q(MapConstCeresVector4(indexer.getCollisionPoseConst(cuboidId, collId)));
                QuaternionT  pose;
                CeresVector3 pos;
                FrameId      outScaledFrameId = partStart * consts.substeps + consts.substeps / 2;
                for (FrameId frameId = partStart; stopCondition->operator()(frameId + 1, partEnd + 1); (*stepFn)(frameId)) {
                    const int subStepEnd   = subStepDir > 0 ? consts.substeps : -1;
                    const int subStepStart = atCollision ? consts.substeps / 2 : (subStepDir > 0 ? 0 : consts.substeps - 1);

                    for (int substep = subStepStart; substep != subStepEnd; substep += subStepDir, outScaledFrameId += subStepDir, atCollision = false) {
//                        CeresScalar evalFrameId = atCollision ? collTime[0] : CeresScalar(outScaledFrameId) / CeresScalar(consts.substeps);
                        CeresScalar evalFrameId = CeresScalar(outScaledFrameId) / CeresScalar(consts.substeps);
                        if (frameId == partStart && !atCollision && substep == subStepEnd - subStepDir) {
                            if ( sgn(evalFrameId-collTime[0]) != subStepDir )
                                evalFrameId = collTime[0] + (evalFrameId + subStepDir / CeresScalar(consts.substeps) - collTime[0]) / 2.;
                        }

                        // position:
                        ParabolaCostFunctor::getPositionAtTime(pos.data(), parabolaRotShared, parabolaA, parabolaTranslation, parabolaFree, evalFrameId - collTime[0]);

                        // pose
                        ceresToQuaternion(q.data(), pose);
                        if ( pose.coeffs().norm() > 1. ) {
                            std::cerr << "[" << __func__ << "] pose norm > 1. :" << pose.coeffs().transpose() << std::endl;
                        }
                        pose.normalize();
                        quaternionToCeres(pose,q.data());
                        outCuboid.addState(outScaledFrameId, PoseLoc(pos.cast<Scalar>(), pose));

                        // linVel:
                        //outCuboid.setLinVel(outScaledFrameId, (v0 + g * ((frameId - frameIds.front()) * k_dt + substep * subDt)).cast<Scalar>());
                        //outCuboid.setLinVel(outScaledFrameId, (v0 + g * ((evalFrameId - frameIds.front()) * consts.k_dt)).cast<Scalar>());
                        {
                            CeresScalar  g[3];
                            CeresVector3 v;
                            GravityCostFunctor::getWorldGravity(parabolaRotShared, parabolaA, consts.k_fps, g);
                            CeresScalar parabolaTime = evalFrameId - collTime[0];
                            LinVelCostFunctor::getLinearVelocityAtTime(parabolaRotShared, parabolaFree, &parabolaTime, CeresScalar(frameIds.front()), consts.k_dt, consts.k_fps, g, v.data());
                            //std::cout << "check linVel: " << v.transpose() << ", g: " << MapConstCeresVector3(g).transpose() << std::endl;
                            outCuboid.setLinVel(outScaledFrameId, v.cast<Scalar>());
                        }

                        // omega:
                        CeresScalar omega[4];
                        estimateOmega(q.data(), momentum.data(), invI.data(), omega);
                        outCuboid.setAngVel(outScaledFrameId, (MapCeresVector4(omega)).template segment<3>(1).cast<Scalar>());
                        if (frameId == partStart && substep == subStepStart)
                        {
                            std::cout << "angvel set: " << outCuboid.getAngVel(outScaledFrameId).transpose() << std::endl;
                        }

                        if (frameId == partStart && substep == subStepStart) {
                            if (partId == collId) {
                                std::cout << "saving before at " << frameId << "." << substep << "(" << evalFrameId << ") for cuboid " << cuboidId << std::endl;
                                out.collisions[collId].states.first[cuboidId] = outCuboid.getState(outScaledFrameId);
                            } else {
                                std::cout << "saving after at " << frameId << "." << substep << "(" << evalFrameId << ") for cuboid " << cuboidId << std::endl;
                                out.collisions[collId].states.second[cuboidId] = outCuboid.getState(outScaledFrameId);
                            }
                            std::cout << "saved " << outCuboid.getState(outScaledFrameId).getLinVel().transpose() << std::endl;
                        }

                        // update pose
                        {
                            //std::cout << "[" << cuboidId << "] fwDt: " << subDt / 2. << std::endl;
                            //std::cout << "[" << cuboidId << "] invI: " << invI.transpose() << std::endl;
                            //std::cout << "[" << cuboidId << "] momentum :" << momentum.transpose() << std::endl;
                            CeresScalar tmp[4];
                            integrateQuaternion(
                                /* [in] startPose: */ q.data(),
                                /* [in]  momentum: */ momentum.data(),
                                /* [in]    dtHalf: */ subStepDir * subDt / 2.,
                                /* [in]      invI: */ invI.data(),
                                /* [out]  outPose: */ tmp,
                                /* [in]  substeps: */ 10
                            );
                            q = (MapCeresVector4(tmp));
                        }

                        // points
                        for (auto const &track2d : tracks2d) {
                            TrackId const& trackId = track2d.getTrackId();
                            if (assignments) {
                                auto it = assignments->find(trackId);
                                if (it != assignments->end() && it->second != cuboidId)
                                    continue;
                            }
                            if (!out.tracks3d)
                                out.tracks3d.reset(new GroupedTracks3d());
                            if (!out.tracks3d->hasLabel(trackId)) {
                                out.tracks3d->addTrack(Track3D(trackId),tracks2d.getGroupId(trackId));
                                //std::cout << "adding track " << trackId << " to group " << tracks2d.getGroupId(trackId) << std::endl;
                            }

                            const CeresScalar *const p = indexer.getPointAddressConst(trackId);
                            CeresScalar pointAtFrameId[3];
                            ceres::QuaternionRotatePoint(q.data(), p, pointAtFrameId);
                            (MapCeresVector3(pointAtFrameId)) += pos;
                            //std::cout << "adding point at frame " << outScaledFrameId << " to track " << trackId << " in group " << out.tracks3d->getGroupId(trackId) << std::endl;
                            Track3D &track3D = out.tracks3d->getTrackByLabel(trackId);
                            if (!track3D.hasPoint(outScaledFrameId))
                                track3D.addPoint(outScaledFrameId, TrackPoint3D::WithNoNormal((MapCeresVector3(pointAtFrameId)).cast<Scalar>()));
                        }
                    } //...substeps
                } //...for frames

                if (partId < static_cast<PartId>(frameIds.size()) - 2)
                {
                    std::cout << "collision time: " << indexer.getCollisionTimeConst(partId)[0] << std::endl;
                }
            } //...for parts
        } //...for collisions
    } //...for cuboids

    // 3D points
    out.points3d.reset(new Tracks3D());
    Tracks3D &outPoints = *out.points3d;
    for (auto const &track2d : tracks2d) {
        const CeresScalar *const p = indexer.getPointAddressConst(track2d.getLabel());
        Track3D point3d(track2d.getLabel());
        point3d.addPoint(FrameId(0), TrackPoint3D::WithNoNormal(MapConstCeresVector3(p).cast<Scalar>()));
        outPoints.addTrack(point3d);
    }

    char name[255];
    for (auto const &collIdAndInfo : out.collisions) {
        Soup::vis::Visualizer<Scalar>::PtrT vis;
        if (doVis)
            vis.reset(new Soup::vis::Visualizer<Scalar>("velDebug"));


        const CollId             &collId        = collIdAndInfo.first;
        const CollisionInfo      &collisionInfo = collIdAndInfo.second;
        const Vector3            collNormal     = out.collisions.at(collId).impulse.normalized();
        Vector3                  collPoint;
        std::unique_ptr<Vector3> offset(nullptr);

        std::vector<const CollisionInfo::CuboidStatesT *> beforeAfter;
        beforeAfter.push_back(&(collisionInfo.states.first));
        beforeAfter.push_back(&(collisionInfo.states.second));
        Vector3  relVel[2] = {Vector3::Zero(), Vector3::Zero()};
        for (int i = 0; i != 2; ++i) {
            const PartId &partId = collId + i;
            Vector3      linMomentum(Vector3::Zero());
            Vector3      angMomentum(Vector3::Zero());
            Scalar       energy(0);
            Vector3      endPoint;

            // center.head<3>().cross(vel.head<3>() * mass)  + momentum.head<3>();
            for (auto const &cuboidIdAndState : *beforeAfter.at(i)) {
                const CuboidId    &cuboidId = cuboidIdAndState.first;
                const PoseLoc     &state    = cuboidIdAndState.second;
                const CeresScalar mass      = out.cuboids->at(cuboidId).getMass();

                //std::cout << "[cub" << cuboidId << "] adding up " << linMomentum.transpose() << " += " << state.getLinVel().transpose() << " * " << mass << "(" << mass * state.getLinVel().transpose() << ") = ";
                linMomentum += mass * state.getLinVel();
                energy += 0.5 * mass * state.getLinVel().dot(state.getLinVel());
                CeresVector3 I;
                out.cuboids->at(cuboidId).getIFromMass(I, mass);
                energy += 0.5 * (I.cast<Scalar>().asDiagonal() * state.getAngVel()).dot(state.getAngVel());
                //std::cout << linMomentum.transpose() << std::endl;

                angMomentum += state.getPosition().cross(state.getLinVel() * mass) + MapConstCeresVector3(indexer.getMomentumConst(cuboidId, partId)).cast<Scalar>();
                if (cuboidId > 1)
                    throw new BundleWithPhysics_EstimateC_TwoCuboidsAssumedException("");

                Vector3 cubColor = cuboidId ? Vector3(0., 0., 1.) : Vector3(1., 0., 0.);
                Vector3 radVel, relCollPoint;
                if (cuboidId)
                    relCollPoint = collisionInfo.states.first.at(0).getPosition() + collisionInfo.relCollPoint - state.getPosition();
                else
                    relCollPoint = collisionInfo.relCollPoint;
//                std::cout << "pos: " << state.getPosition().transpose() << std::endl;
//                std::cout << "relCollPoint: " << relCollPoint.transpose() << std::endl;

                if (cuboidId == i) { // c = (v'_b - v'_a) / (v_a - v_b) // i == cuboidId is true, for A before and B after: "-="
                    radVel = getVelAtCollPoint(state.getTransform().matrix(), relCollPoint, state.getAngVel());
                    //std::cout << "[" << (i?"AFTER":"BEFORE") << " cub" << cuboidId << "]:\n";
                    //std::cout << "\trelVel[" << i << "]: " << relVel[i].transpose() << " -= " << state.getLinVel().transpose() << " + " << radVel.transpose();
                    relVel[i] -= state.getLinVel() + radVel;
//                    std::cout << "radVel: " << radVel.transpose() << std::endl;
                    //std::cout << " = " << relVel[i].transpose() << std::endl;
                } else {
                    radVel = getVelAtCollPoint(state.getTransform().matrix(), relCollPoint, state.getAngVel());
                    //std::cout << "[" << (i?"AFTER":"BEFORE") << " cub" << cuboidId << "]:\n";
                    //std::cout << "\trelVel[" << i << "]: " << relVel[i].transpose() << " += " << state.getLinVel().transpose() << " + " << radVel.transpose();
                    relVel[i] += state.getLinVel() + radVel;
//                    std::cout << "radVel: " << radVel.transpose() << std::endl;
                    //std::cout << " = " << relVel[i].transpose() << std::endl;
                }
                collPoint = state.getPosition() + relCollPoint;
                if (!offset)
                    offset.reset(new Vector3(collPoint));

                if (!cuboidId) {
                    endPoint = collPoint;
                }

                sprintf(name, "vel%d_%d", i, cuboidId);
                //vis.addArrow( state.getPosition(), state.getPosition() + state.getLinVel()/20., Vector3(!cuboidId, i, cuboidId), name );
                //vis.addArrow(endPoint - *offset, endPoint + ((cuboidId == i) ? -1. : 1.) * state.getLinVel() / 20. - *offset, Vector3(!cuboidId, i * 0.5, cuboidId), std::string("cum") + name, .05);
                endPoint += ((cuboidId == i) ? -1. : 1.) * state.getLinVel() / 20.;

                sprintf(name, "radVel%d_%d", i, cuboidId);
                //vis.addArrow(state.getPosition() + relCollPoint, state.getPosition() + relCollPoint + radVel/20., Vector3(!cuboidId, i, cuboidId), name );

                //vis.addArrow(endPoint - *offset, endPoint + ((cuboidId == i) ? -1. : 1.) * radVel / 20. - *offset, Vector3(!cuboidId, i * 0.5, cuboidId), std::string("cum") + name);
                endPoint += ((cuboidId == i) ? -1. : 1.) * radVel / 20.;

                //vis.addArrow( state.getPosition(), state.getPosition() + relCollPoint, Vector3::Zero(), std::string("x") + name );

                // visDebug
                if (i && doVis)
                {
                    if (randf() > 0.33)
                        continue;
                    sprintf(name, "cub%d_bef%d", cuboidId, i);
                    drawCuboid(*vis, Eigen::Translation<Scalar, 3>(-*offset) * state.getTransform() * out.cuboids->at(cuboidId).getSizeTransform(), cubColor, name);
                    if (!cuboidId)
                    {
                        vis->addLine(state.getPosition() + relCollPoint - collNormal - *offset, state.getPosition() + relCollPoint + collNormal - *offset, Vector3::Zero(), "normal", 2., 1.);
                    }
                }
            }
//            std::cout << "linMomentum[" << (i ? "AFTER" : "BEFORE") << "]: " << linMomentum.transpose() << std::endl;
//            std::cout << "angMomentum[" << (i ? "AFTER" : "BEFORE") << "]: " << angMomentum.transpose() << std::endl;
//            std::cout << "relVel[" << (i ? "AFTER" : "BEFORE") << "]: " << relVel[i].transpose() << std::endl;
            std::cout << "kineticE[" << (i ? "AFTER" : "BEFORE") << "]: " << energy << std::endl;

        } //...before and after

        if (doVis)
        {
            vis->addArrow(collPoint - *offset, collPoint + relVel[0].dot(collNormal) * collNormal / 20. - *offset, Vector3(1., .8, .8), "dotBefore");
            vis->addArrow(collPoint - *offset, collPoint + relVel[1].dot(collNormal) * collNormal / 20. - *offset, Vector3(.8, 1., .8), "dotAfter");
        }
        CeresScalar c = relVel[1].dot(collNormal) / relVel[0].dot(collNormal);
        std::cout << "c: " << relVel[1].dot(collNormal) << "/" << relVel[0].dot(collNormal)
                  << " = " << c
                  << std::endl;
        out.collisions.at(collId).cor = c;

        {
            MapConstCeresVector3 trA(indexer.getParabolaTranslationConst(0, 0));
            MapConstCeresVector3 trB(indexer.getParabolaTranslationConst(1, 0));
            std::cout << "collPoint: " << collPoint.transpose() << ", trA: " << trA.transpose() << ", trB: " << trB.transpose() << std::endl;
            //std::cout << "(x2-x1) x (x1-p): " << (trB - trA).cross(trA - collPoint.cast<CeresScalar>()) / (trB - trA).norm() << std::endl;
        }
        if (doVis)
        {
            vis->setCameraFocalPoint(Eigen::Vector3d::Zero());
            //vis.addCoordinateSystem(0.5);

            //EnergyPlotter::getInstance().plot();

            vis->spin();
        }
    } //...for collisions
} //...output
} //...ns bundle_physics
} //...ns tracking