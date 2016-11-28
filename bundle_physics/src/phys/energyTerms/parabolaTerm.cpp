//
// Created by bontius on 02/05/16.
//

#include "tracking/phys/energyTerms/parabola2dTerm.h"
#include "tracking/phys/energyTerms/impl/parabolaTerm.hpp"
#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/annot/cuboid.h"
#include "tracking/phys/parabola.h"
#include "tracking/phys/weights.h"
#include "tracking/phys/consts.h"
#include "tracking/phys/partUtil.h"
#include "ceres/functorInfo.h"

namespace tracking {
namespace bundle_physics {

void addParabolaTerms(
    ceres::Problem               &       problem,
    PhysIndexer                  &       indexer,
    ceres::FunctorInfosT         &       costFunctors,
    FrameIdsT               const&       frameIds,
    Weights                 const&       weights,
    Mapper                  const&       mapper,
    CuboidsT                const&       cuboids,
    Consts                  const&       consts,
    BundleWithPhysicsResult const* const initial,
    bool                    const        use2dTerm
) {

    using ceres::CeresScalar;
    using ceres::CeresVector3;
    using ceres::MapCeresVector3;
    using ceres::MapConstCeresVector3;
    char name[255];

    size_t termCount(0);
    ceres::LossFunctionWrapper *const parabolaLossNormalizer(new ceres::LossFunctionWrapper(new ceres::ScaledLoss(NULL,1.,ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP));
    ceres::LossFunctionWrapper *const parabolaLossNormalizerHuber(new ceres::LossFunctionWrapper(new ceres::ScaledLoss(NULL,1.,ceres::TAKE_OWNERSHIP), ceres::TAKE_OWNERSHIP));

    CeresScalar* const parabolaRotShared = indexer.getParabolaRotationShared();
    CeresScalar* const parabolaA         = indexer.getParabolaSquaredParam();

    for ( CuboidId cuboidId = 0; cuboidId != CuboidId(cuboids.size()); ++cuboidId ) {
        Cuboid const& cuboid = cuboids.at(cuboidId);
        for (auto const &frameAndState : cuboid.getStates()) {
            const FrameId &frameId = frameAndState.first;
            if (!frameAndState.second.hasPos()) {
                std::cerr << "[" << __func__ << "] skipping frame without position: " << frameId << std::endl;
                continue;
            }

            // deduce parabolaId of cuboid
            const PartId partId( getPartId(frameId, frameIds) );
            const CollId collId( std::max(0,int(partId)-1) );

//                auto T = cuboid.getTransform4x4(frameId);
            CeresScalar* const parabolaTranslation  = indexer.getParabolaTranslation (cuboidId, collId );
            CeresScalar* const parabolaFreeParams   = indexer.getParabolaFreeParams  (cuboidId, partId);
            CeresScalar* const collTime             = indexer.getCollisionTime       (collId);

            if (!(weights.fixFlags & Weights::FIX_PARABOLAS)) {
                if (use2dTerm) {
                    {
                        ceres::CostFunction *costFunction = Parabola2DFunctor::Create(weights.observedPosWeight, frameId, /* xyz: */ cuboid.getPosition(frameId), mapper);
                        problem.AddResidualBlock(costFunction, parabolaLossNormalizer, parabolaRotShared, parabolaA, parabolaTranslation, parabolaFreeParams, collTime);
                        ++termCount;

                        sprintf(name, "2DParabolaError_time%u", static_cast<unsigned>(frameId));
                        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {parabolaRotShared, parabolaA, parabolaTranslation, parabolaFreeParams, collTime}));
                    }
                    #if 1
                    {
                        ceres::CostFunction *costFunction = DepthPriorFunctor::Create(weights.observedPosWeight * 0.316, frameId, /* xyz: */ cuboid.getPosition(frameId), mapper);
                        problem.AddResidualBlock(costFunction, parabolaLossNormalizerHuber, parabolaRotShared, parabolaA, parabolaTranslation, parabolaFreeParams, collTime);

                        sprintf(name, "DepthPrior_time%u", static_cast<unsigned>(frameId));
                        costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {parabolaRotShared, parabolaA, parabolaTranslation, parabolaFreeParams, collTime}));
                    }
                    #endif
                } else {
                    //if (cuboids.size() > 1 ) {
                    std::cerr << "[" << __FILE__ << ":" << __LINE__ << "] "
                              << "add --para2d to the calls" << std::endl;
                    throw new std::runtime_error("");
                    //}
                    ceres::CostFunction *costFunction = ParabolaCostFunctor::Create(weights.observedPosWeight, frameId, /* xyz: */ cuboid.getPosition(frameId));
                    problem.AddResidualBlock(costFunction, parabolaLossNormalizer, parabolaRotShared, parabolaA, parabolaTranslation, parabolaFreeParams, collTime);
                    ++termCount;

                    sprintf(name, "ParabolaError_time%u", static_cast<unsigned>(frameId));
                    costFunctors.emplace_back(ceres::FunctorInfo(name, costFunction, {parabolaRotShared, parabolaA, parabolaTranslation, parabolaFreeParams, collTime}));
                } //...use2dTerm
            } //...fixParabolas
        } //...for states

        // init parabolas
        for (PartId partId = consts.firstPartId; partId <= consts.lastPartId; ++partId) {
            if ((weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_PARABOLAS) &&
                (weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_TRANSLATIONS))
                throw new Weights_InitDiscrepancyException("");

            const CollId collId(std::max(0, partId - 1));
            CeresScalar *const parabolaRotFree     = indexer.getParabolaRotationFree(cuboidId, partId);
            CeresScalar *const parabolaTranslation = indexer.getParabolaTranslation(cuboidId, collId);
            CeresScalar *const parabolaSParam      = indexer.getParabolaSParam(cuboidId, partId);
            CeresScalar *const parabolaBParam      = indexer.getParabolaBParam(cuboidId, partId);

            // init parabola to rotation pointing away from camera with Euler angles convention YXY
            parabolaRotFree[0]   = 20; // theta_{y0}
            parabolaRotShared[0] = 0.; // theta_x
            parabolaRotShared[1] = 0.; // theta_{y1}
            if (weights.fixFlags & Weights::FIX_FLAGS::FIX_GRAVITY)
                problem.SetParameterBlockConstant(parabolaRotShared);

            if (weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_PARABOLAS && initial && initial->hasParabola(cuboidId, partId)) {
                Parabola const &parabola = initial->parabolas.at(cuboidId).at(partId);
                parabolaRotFree[0]                   = parabola.rotY0;
                parabolaRotShared[0]                 = initial->rotX;
                parabolaRotShared[1]                 = initial->rotY1;
                indexer.getParabolaSquaredParam()[0] = initial->a;
                parabolaSParam[0]                    = parabola.s;
                parabolaBParam[0]                    = parabola.b;
                (MapCeresVector3(parabolaTranslation)) = MapConstCeresVector3(parabola.getTranslation().data());

                // log
                std::cout << "[" << __func__ << "] init parabola translation cub" << cuboidId << ", part" << partId << ": "
                          << "\n\ttranslation: " << parabolaTranslation[0] << "," << parabolaTranslation[1] << "," << parabolaTranslation[2]
                          << "\n\ty0: " << parabolaRotFree[0] << ", s: " << parabolaSParam[0] << ", b: " << parabolaBParam[0]
                          << std::endl;

                if (weights.fixFlags & Weights::FIX_FLAGS::FIX_PARABOLAS) {
                    problem.SetParameterBlockConstant(parabolaTranslation);
                    problem.SetParameterBlockConstant(indexer.getParabolaFreeParams(cuboidId, partId));
                    //problem.SetParameterBlockConstant(parabolaSParam);
                    //problem.SetParameterBlockConstant(parabolaBParam);
                }
            } else if ((weights.initFlags & Weights::INIT_FLAGS::USE_INPUT_TRANSLATIONS) && initial && initial->hasParabola(cuboidId, partId)) {
                std::cout << "init parabola translation cub" << cuboidId << ", part" << partId << ": ";
                Parabola const &parabola = initial->parabolas.at(cuboidId).at(partId);
                (MapCeresVector3(parabolaTranslation)) = MapConstCeresVector3(parabola.getTranslation().data());
                std::cout << parabolaTranslation[0] << "," << parabolaTranslation[1] << "," << parabolaTranslation[2] << std::endl;
            } else {
                std::cerr << "not using parabola initialization" << std::endl;
                // init offset to half meter in front of camera
                (MapCeresVector3(parabolaTranslation)) = CeresVector3(0., 0., 0.5);

                parabolaBParam[0] = -0.051;
                // init parabola x stretch to 1/fps
                parabolaSParam[0] = consts.k_dt;
            }
        } //...init parabolas
    } //...cuboids

    parabolaLossNormalizer->Reset(
        new ceres::ScaledLoss(nullptr, 1. / static_cast<float>(termCount), ceres::TAKE_OWNERSHIP),
        ceres::TAKE_OWNERSHIP);
    parabolaLossNormalizerHuber->Reset(
        new ceres::ScaledLoss(new ceres::HuberLoss(10.), 1. / static_cast<float>(termCount), ceres::TAKE_OWNERSHIP),
        ceres::TAKE_OWNERSHIP);
} //...addParabolaTerms


namespace _h {
typedef ceres::Jet<double,21> Jet21T;
typedef ceres::Jet<double,10> Jet10T;
typedef ceres::Jet<double,4> Jet4T;
typedef ceres::Jet<double,15> Jet15T;
typedef ceres::Jet<double,17> Jet17T;
}

template void ParabolaCostFunctor::getPositionAtTime(double* x,
                                                     double const* rotG,
                                                     double const* a,
                                                     double const* translation,
                                                     double const* const free,
                                                     double const& time);

template void ParabolaCostFunctor::getPositionAtTime(_h::Jet21T* x,
                                                     _h::Jet21T const* rotG,
                                                     _h::Jet21T const* a,
                                                     _h::Jet21T const* translation,
                                                     _h::Jet21T const* const free,
                                                     _h::Jet21T const& time);

template void ParabolaCostFunctor::getPositionAtTime(_h::Jet10T* x,
                                                     _h::Jet10T const* rotG,
                                                     _h::Jet10T const* a,
                                                     _h::Jet10T const* translation,
                                                     _h::Jet10T const* const free,
                                                     _h::Jet10T const& time);

template void ParabolaCostFunctor::getPositionAtTime(_h::Jet4T* x,
                                                     _h::Jet4T const* rotG,
                                                     _h::Jet4T const* a,
                                                     _h::Jet4T const* translation,
                                                     _h::Jet4T const* const free,
                                                     _h::Jet4T const& time);

template void ParabolaCostFunctor::getPositionAtTime(_h::Jet15T* x,
                                                     _h::Jet15T const* rotG,
                                                     _h::Jet15T const* a,
                                                     _h::Jet15T const* translation,
                                                     _h::Jet15T const* const free,
                                                     _h::Jet15T const& time);

template void ParabolaCostFunctor::getPositionAtTime(_h::Jet17T* x,
                                                     _h::Jet17T const* rotG,
                                                     _h::Jet17T const* a,
                                                     _h::Jet17T const* translation,
                                                     _h::Jet17T const* const free,
                                                     _h::Jet17T const& time);

template ceres::CostFunction* ParabolaCostFunctor::Create(double,
                                                          unsigned int,
                                                          Eigen::Block<Eigen::Matrix<float, 4, 4, 0, 4, 4>,
                                                                       3,
                                                                       1,
                                                                       false> const&);
} //...ns bundle_physics
} //...ns tracking
