//
// Created by bontius on 01/04/16.
//

//#include "tracking/phys/initialize/initialize.h"

#include "tracking/phys/initialize/assignments.h"
#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"
#include "tracking/phys/physFunctorInfo.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/typedefsGeometry.h"
#include "tracking/phys/partUtil.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/weights.h"
#include "tracking/phys/consts.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/util/util.h"
#include "tracking/common/maths/quaternion.h"
#include "ceres/ceres.h"

namespace tracking {
  namespace bundle_physics{

    void initializeParabolas(ceres::Problem &problem, PhysIndexer &indexer, Weights const& weights, Consts const& consts, int const nCuboids, BundleWithPhysicsResult const*const initial) {
        using ceres::CeresScalar;
        using ceres::CeresVector3;
        using ceres::MapCeresVector3;
        using ceres::MapConstCeresVector3;

        if (initial) {

            // rotG
            std::cout << "[" << __func__ << "] using input gravity rotation" << std::endl;
            CeresScalar *const parabolaRotShared = indexer.getParabolaRotationShared();
            parabolaRotShared[0] = initial->rotX; // theta_x
            parabolaRotShared[1] = initial->rotY1; // theta_{y1}
            std::cout << "[" << __func__ << "] fixing gravity rotation" << std::endl;
            problem.SetParameterBlockConstant(parabolaRotShared);

            for (int cuboidId = 0; cuboidId != nCuboids; ++cuboidId) {
                for (PartId partId = consts.firstPartId; partId <= consts.lastPartId; ++partId) {
                    CollId const collId(std::max(0, partId - 1));
                    CeresScalar *const parabolaRotFree     = indexer.getParabolaRotationFree(cuboidId, partId);
                    CeresScalar *const parabolaTranslation = indexer.getParabolaTranslation (cuboidId, collId);
                    CeresScalar *const parabolaBParam      = indexer.getParabolaBParam      (cuboidId, partId);
                    CeresScalar *const parabolaSParam      = indexer.getParabolaSParam      (cuboidId, partId);

                    Parabola const& parabola = initial->parabolas.at(cuboidId).at(partId);
                    // init parabola to rotation pointing away from camera with Euler angles convention YXY
                    parabolaRotFree[0] = parabola.rotY0; // theta_{y0}

                    // init offset to half meter in front of camera
                    (MapCeresVector3(parabolaTranslation)) = (MapConstCeresVector3(parabola.getTranslation().data()));

                    // parabolaBParam
                    parabolaBParam[0] = parabola.b;
                    problem.SetParameterLowerBound(parabolaRotFree, 1, weights.bounds.parabolaB.lower); // -0.1
                    problem.SetParameterUpperBound(parabolaRotFree, 1, weights.bounds.parabolaB.upper); // 0.1

                    // parabolaSParam
                    parabolaSParam[0] = parabola.s;
                    problem.SetParameterLowerBound(parabolaRotFree, 2, weights.bounds.parabolaS.lower); // -0.1
                    problem.SetParameterUpperBound(parabolaRotFree, 2, weights.bounds.parabolaS.upper); // 0.1

                    for (int dim = 0; dim != 3; ++dim) {
                        problem.SetParameterLowerBound(parabolaTranslation, dim, weights.bounds.translation.lower(dim));
                        problem.SetParameterUpperBound(parabolaTranslation, dim, weights.bounds.translation.upper(dim));
                    }
                } //...for parts
            } //...for cuboids
        } else {

            // init parabolas
            CeresScalar *const parabolaRotShared = indexer.getParabolaRotationShared();
            parabolaRotShared[0] = 0.; // theta_x
            parabolaRotShared[1] = 0.; // theta_{y1}
            problem.SetParameterBlockConstant(parabolaRotShared);
            //CeresScalar* const parabolaA         = indexer.getParabolaSquaredParam();

            for (int cuboidId = 0; cuboidId != nCuboids; ++cuboidId) {
                for (PartId partId = consts.firstPartId; partId <= consts.lastPartId; ++partId) {
                    if ((weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_PARABOLAS) &&
                        (weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_TRANSLATIONS))
                        throw new Weights_InitDiscrepancyException("");

                    CollId const collId(std::max(0, partId - 1));
                    CeresScalar *const parabolaRotFree     = indexer.getParabolaRotationFree(cuboidId, partId);
                    CeresScalar *const parabolaTranslation = indexer.getParabolaTranslation(cuboidId, collId);
                    CeresScalar *const parabolaBParam      = indexer.getParabolaBParam(cuboidId, partId);
                    CeresScalar *const parabolaSParam      = indexer.getParabolaSParam(cuboidId, partId);

                    // init parabola to rotation pointing away from camera with Euler angles convention YXY
                    parabolaRotFree[0] = 20.; // theta_{y0}

                    if (weights.fixFlags & Weights::FIX_FLAGS::FIX_GRAVITY)
                        problem.SetParameterBlockConstant(parabolaRotShared);

                    // init offset to half meter in front of camera
                    (MapCeresVector3(parabolaTranslation)) = CeresVector3(0., 0., 4.1);

                    parabolaBParam[0] = -0.051;
                    // init parabola x stretch to 1/fps
                    parabolaSParam[0] = consts.k_dt;

                    // parabolaRotFree
                    problem.SetParameterLowerBound(parabolaRotFree, 0, weights.bounds.parabolaRotation.lower(0)); // -360
                    problem.SetParameterUpperBound(parabolaRotFree, 0, weights.bounds.parabolaRotation.upper(0)); // +360

                    // parabolaBParam
                    problem.SetParameterLowerBound(parabolaRotFree, 1, weights.bounds.parabolaB.lower); // -0.1
                    problem.SetParameterUpperBound(parabolaRotFree, 1, weights.bounds.parabolaB.upper); // 0.1

                    // parabolaSParam
                    problem.SetParameterLowerBound(parabolaRotFree, 2, weights.bounds.parabolaS.lower); // -0.1
                    problem.SetParameterUpperBound(parabolaRotFree, 2, weights.bounds.parabolaS.upper); // 0.1

                    for (int dim = 0; dim != 3; ++dim) {
                        problem.SetParameterLowerBound(parabolaTranslation, dim, weights.bounds.translation.lower(dim));
                        problem.SetParameterUpperBound(parabolaTranslation, dim, weights.bounds.translation.upper(dim));
                    }
                } //...for parts
            } //...for cuboids
        }
    } //...initializeParabolas()

    void initializeMasses(ceres::Problem &problem, PhysIndexer &indexer, Weights const& weights, int const nCuboids) {
        using ceres::CeresScalar;

        for (int cuboidId = 0; cuboidId != nCuboids; ++cuboidId) {
            bool        fixed(false);
            CeresScalar *mass = indexer.getMass(cuboidId, &fixed);
            mass[0] = 1.;

            if (weights.solveFlags & Weights::SOLVE) { // mass has not been added to problem, so can't fix it
                if (fixed)
                    problem.SetParameterBlockConstant(mass);
                else {
                    problem.SetParameterLowerBound(mass, 0, weights.bounds.mass.lower); // 1.e-5
                    problem.SetParameterUpperBound(mass, 0, weights.bounds.mass.upper); // 10.
                }
            }
        } //...for cuboids
    } //...initializeMasses()

    void initializeCollisionTime(ceres::Problem &problem, PhysIndexer &indexer, FrameIdsT const& frameIds, Weights const& weights, Consts const& consts) {
        using ceres::CeresScalar;

        // init poses and time
        for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
            //PartId const partIdBefore = getPartIdBefore(collId);
            //PartId const partIdAfter  = getPartIdAfter (collId);
            // init collision time
            CeresScalar *collisionTime = indexer.getCollisionTime(collId);

            collisionTime[0] = frameIds.at(collId + 1) + 0.5;

            if (weights.fixFlags & Weights::FIX_FLAGS::FIX_COLLTIMES) {
                std::cout << "fixing time " << collId << std::endl;
                problem.SetParameterBlockConstant(collisionTime);
            } else {
                problem.SetParameterLowerBound(collisionTime, 0, getPrev(frameIds.at(collId + 1)));
                problem.SetParameterUpperBound(collisionTime, 0, getNext(getNext(frameIds.at(collId + 1))));
            }
            std::cout << "collisionTime inited to " << collisionTime[0] << ", and bounded:" << frameIds.at(collId + 1) << ".." << getNext(frameIds.at(collId + 1)) << std::endl;
        } //...for collisions
    } //...initializeCollisionTime()

    DEFINE_EXCEPTION(NoInputCollPoint)
    void initCollPoint2(
        ceres::Problem                &      problem,
        PhysIndexer                   &      indexer,
        const Weights                 &      weights,
        const std::vector<CuboidId>   &      participants,
        const Consts                  &      consts,
        const BundleWithPhysicsResult *const initial)
    {
        //using _Scalar = tracking::bundle_physics::Scalar;
        using ceres::CeresScalar;
        using ceres::CeresVector3;
        using ceres::MapCeresVector3;
        using ceres::MapConstCeresVector3;

        for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
            CeresScalar *const collPoint = indexer.getCollisionPoint(collId);

            if (weights.initFlags & bundle_physics::Weights::INIT_FLAGS::USE_INPUT_COLLPOINTS) {
                if (!initial)
                    throw new NoInputCollPointException("");

                (MapCeresVector3(collPoint)) = initial->collisions.at(collId).relCollPoint.cast<CeresScalar>();
                std::cout << "inited collPoint to " << MapCeresVector3(collPoint).transpose() << std::endl;
            }
            else
            {
                const CuboidId  objA = getParticipant(0, collId, participants);
                const CuboidId  objB = getParticipant(1, collId, participants);
                MapCeresVector3 parabolaTranslationA(indexer.getParabolaTranslation(objA, collId));
                MapCeresVector3 parabolaTranslationB(indexer.getParabolaTranslation(objB, collId));

                (MapCeresVector3(collPoint)) = (parabolaTranslationB - parabolaTranslationA) / 2.;

                std::cout << "inited collPoint to " << MapCeresVector3(collPoint).transpose()
                << ", from tA: " << parabolaTranslationA.transpose()
                << ", and tB: " << parabolaTranslationB.transpose()
                << std::endl;
#if 0
                // debug
                    {
                        Soup::vis::Visualizer<_Scalar> v2("initCollPoint");
                        v2.addSphere(parabolaTranslationA.cast<_Scalar>(), 0.01, Vector3(1., 0., 0.), "trA");
                        v2.addSphere(parabolaTranslationB.cast<_Scalar>(), 0.01, Vector3(0., 1., 0.), "trB");
                        v2.addLine((parabolaTranslationA + MapConstCeresVector3(collPoint)).cast<_Scalar>(),
                                    parabolaTranslationA.cast<_Scalar>(), Vector3::Zero(), "cp");

                        v2.spin();
                    }
#endif
            }

            if (weights.fixFlags & Weights::FIX_FLAGS::FIX_COLLPOINT) {
                std::cerr << "fixing collPoint" << std::endl;
                problem.SetParameterBlockConstant(collPoint);
            }
            for (int dim = 0; dim != 3; ++dim) {
                problem.SetParameterLowerBound(collPoint, dim, weights.bounds.objectSize.lower(dim));
                problem.SetParameterUpperBound(collPoint, dim, weights.bounds.objectSize.upper(dim));
            }
        } //...for collisions
    } //...initCollPoint

    void initializePoses(ceres::Problem &problem, PhysIndexer &indexer, Weights const &weights, Consts const& consts, int const nCuboids) {
        using ceres::CeresScalar;

        CollId const collId = consts.firstCollId;
        for (CuboidId cuboidId = 0; cuboidId != nCuboids; ++cuboidId) {
            // initialize collision pose
            CeresScalar *const poseAddress = indexer.getCollisionPose(cuboidId, collId);
            QuaternionT initPose(QuaternionT::Unit());
            quaternionToCeres(initPose, poseAddress);

            if (weights.fixFlags & Weights::FIX_POSE) {
                std::cout << "fixing pose for " << cuboidId << " and coll " << collId << std::endl;
                problem.SetParameterBlockConstant(poseAddress);
            }
        } //...for cuboids
    } //...initializePoses()

    void initializeMomenta(ceres::Problem &problem, PhysIndexer &indexer, Weights const &weights, Consts const& consts, int const nCuboids) {
        using ceres::CeresScalar;
        using ceres::CeresVector3;
        using ceres::MapCeresVector3;

        CollId const collId = consts.firstCollId;
        PartId const partIdBefore = getPartIdBefore(collId);
        PartId const partIdAfter  = getPartIdAfter (collId);
        for (CuboidId cuboidId = 0; cuboidId != nCuboids; ++cuboidId) {
            // before collision
            {
                CeresScalar *const momentumAddress = indexer.getMomentum(cuboidId, partIdBefore);
                //(MapCeresVector3(momentumAddress)) = currI * omega0.coeffs().head<3>().cast<CeresScalar>();
                (MapCeresVector3(momentumAddress)) << randf(0.01) - 0.005, randf(0.01) - 0.005, randf(0.01) - 0.005;
                //(MapCeresVector3(momentumAddress)) << 0.,0.,0.;
                std::cout << "inited to momentum " << (MapCeresVector3(momentumAddress)).transpose()
                //<< ", from omega " << omega0.coeffs().transpose()
                << std::endl;
            }
            // after collision
            {
                CeresScalar *const momentumAddress = indexer.getMomentum(cuboidId, partIdAfter);
                //(MapCeresVector3(momentumAddress)) = currI * omega1.coeffs().head<3>().cast<CeresScalar>();
                (MapCeresVector3(momentumAddress)) << randf(0.01) - 0.005, randf(0.01) - 0.005, randf(0.01) - 0.005;
                //(MapCeresVector3(momentumAddress)) << 0.,0.,0.;
                std::cout << "inited to momentum " << (MapCeresVector3(momentumAddress)).transpose()
                //<< ", from omega " << omega1.coeffs().transpose()
                << std::endl;
                for (int dim = 0; dim != 3; ++dim) {
                    problem.SetParameterLowerBound(momentumAddress, dim, weights.bounds.momentum.lower(dim));
                    problem.SetParameterUpperBound(momentumAddress, dim, weights.bounds.momentum.upper(dim));
                }
            }
        } //...for cuboids
    } //...initializeMomenta()

    void initializePoints(ceres::Problem &/*problem*/, PhysIndexer &indexer, Weights const& /*weights*/, PhysFunctorInfosT const& functorInfos,
        Consts const& /*consts*/, GroupedTracks2d const& tracks2d, TracksToCuboidsT const& assignments, CollId const collId, FrameIdsT const& frameIds,
        Mapper const& mapper) {
        using ceres::MapCeresVector3;
        using ceres::CeresVector3;
        using ceres::CeresScalar;

        for (std::shared_ptr<PhysFunctorInfo> const& functorInfo : functorInfos) {
            std::shared_ptr<TrackPhysFunctorInfo> tpf = std::dynamic_pointer_cast<TrackPhysFunctorInfo>(functorInfo);
            if (tpf) {
                for (TrackId const& trackId : tpf->getTrackIds()) {
                    ceres::MapCeresVector3  ad(indexer.getPointAddress(trackId));
                    Track2D     const&      track2D  = tracks2d.getTrackByLabel(trackId);
                    CuboidId    const       cuboidId = assignments.at(trackId);
                    FrameId     const       frameId  = track2D.getPoints().begin()->first;
                    PartId      const       partId   = getPartId(frameId,frameIds);
                    ceres::MapConstCeresVector3 translation(indexer.getParabolaTranslationConst(cuboidId,collId));
                    ceres::CeresVector3 x;
                    ParabolaCostFunctor::getPositionAtTime( /* out: */ x.data(),
                        indexer.getParabolaRotationSharedConst(), indexer.getParabolaSquaredParamConst(),
                        translation.data(), indexer.getParabolaFreeParamsConst(cuboidId,partId),
                        CeresScalar(frameId) - *indexer.getCollisionTimeConst(collId));

                    // hallucinate point in global coordinates by giving it the depth of the initialized parabola depth
                    auto const& p3 = mapper.to3D(track2D.getPoint(frameId),translation(2)-0.1);

                    ad = p3.cast<ceres::CeresScalar>() - x;
                    //std::cout << "[" << __func__ << "] inited point to " << ad.transpose() << std::endl;
//                    for (int dim = 0; dim != 3; ++dim) {
//                        problem.SetParameterLowerBound(ad, dim, -weights.bounds.objectSize.lower.maxCoeff());
//                        problem.SetParameterUpperBound(ad, dim, weights.bounds.objectSize.maxCoeff());
//                    }
                }
            }
        }
#if 0
        for (auto it = std::begin(indexer.getLinPointIds()); it != std::end(indexer.getLinPointIds()); ++it) {
            TrackId const& trackId = it->first;
            // get parabola position
            //CeresVector3 pos;
//            ParabolaCostFunctor::getPositionAtTime(pos.data(), indexer.getParabolaRotationShared(), indexer.getParabola,
//                indexer.getParabolaTranslation(cuboidId,collId), parabolaTranslation, indexer.getParabolaFreeParams(cuboidId,), evalFrameId - collTime[0]);
            // subtract from point position
            CeresScalar *const ad = indexer.getPointAddress(trackId);
            MapCeresVector3 pointAddress(indexer.getPointAddress(trackId));
            pointAddress << randf(),randf(),randf();
            for (int dim = 0; dim != 3; ++dim) {
                problem.SetParameterLowerBound(ad, dim, weights.bounds.objectSize.lower(dim));
                problem.SetParameterUpperBound(ad, dim, weights.bounds.objectSize.upper(dim));
            }
        }
#endif
    } //...initializePoints()

    void initialize(
        ceres::Problem               &       problem,
        PhysIndexer                  &       indexer,
        PhysFunctorInfosT       const&       costFunctors,
        FrameIdsT               const&       frameIds,
        GroupedTracks2d         const&       tracks2d,
        TracksToCuboidsT        const&       assignments,
        Weights                 const&       weights,
        Consts                  const&       consts,
        Mapper                  const&       mapper,
        int                     const        nCuboids,
        std::vector<CuboidId>   const&       /*participants*/,
        CollId                  const        collId,
        BundleWithPhysicsResult const* const initial) {
        initializeParabolas    (problem,indexer,weights,consts,nCuboids,initial);
        initializeMasses       (problem,indexer,weights,nCuboids);
        initializeCollisionTime(problem,indexer,frameIds,weights,consts);
        initializePoses        (problem,indexer,weights,consts,nCuboids);
        initializeMomenta      (problem,indexer,weights,consts,nCuboids);
        initGravityTerm        (problem,indexer,consts.k_fps);
        //initCollPoint2       (problem,indexer,weights,participants,consts,initial);
        initializePoints       (problem,indexer,weights,costFunctors,consts,tracks2d,assignments,collId,frameIds,mapper);
    } //...initialize()

  } //...ns bundle_physics
} //...ns tracking