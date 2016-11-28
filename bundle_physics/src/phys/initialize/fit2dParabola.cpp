//
// Created by bontius on 04/04/16.
//

#include "tracking/phys/initialize/fitParabolas.h"
#include "tracking/phys/initialize/parabola2d.h"
#include "tracking/phys/initialize/assignments.h"
#include "tracking/phys/energyTerms/impl/gravityTerm.hpp"
#include "tracking/phys/weights.h"

#include "tracking/phys/physIndexer.h"
#include "tracking/phys/ceresUtil.h"
#include "tracking/phys/partUtil.h"
#include "tracking/phys/typedefsGeometry.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/bundleWithPhysics.h"
#include "tracking/common/mapper.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/util/colors.h"
#include "tracking/common/util/util.h"
#include "tracking/common/io/os.h"
#include "ceres/problem.h"
#include "ceres/ceres.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core.hpp"
#include <list>

namespace tracking {
namespace bundle_physics {
#if 0
struct Parabola2dFitFunctor {
        public:
            Parabola2dFitFunctor(double t,double x, double y) : _t(t), _x(x), _y(y) {}
            enum { NUM_RESIDUALS = 2 };
            template <typename T>
            bool operator()(T const a[4], T* residuals) const {
                residuals[0] = T(_y) - a[2] * T(_t) * T(_t) - a[1] * T(_t) - a[0];
                residuals[1] = T(_x) - a[3] * T(_t);
                return true;
            }
        protected:
            double _t,_x,_y;
    }; //...FitFunctor
#endif

#if 0
struct RepulseFunctor {
    public:
        template <typename _Matrix3, typename _Vector3>
        RepulseFunctor(double const sqrtWeight, _Matrix3 const& intr,  _Vector3 const& size0, _Vector3 const* size1 = nullptr )
            : _sqrtWeight(sqrtWeight),
              _fx( intr(0,0) ),
              _fy( intr(1,1) ),
              _halfDiagSum( (size1 ? (size0.norm() + size1->norm())/2. : (size0.norm()/2.)) * 0.85)
        {}

        enum { NUM_RESIDUALS = 1 };

        /** \param[in ] free Contains angle y0, linear b, scale s */
        template <typename T>
        bool operator()(T const rotG[2], T const a[1], T const txy0[2], T const txy1[2],
                        T const tz[1], T const free0[3], T const free1[3], T const collTime[1],
                        T* residuals) const {

//                T const translation0[3] = {txy0[0], txy0[1], tz[0]};
//                T const translation1[3] = {txy1[0], txy1[1], tz[0]};
//                T x0[3], x1[3];
//                ParabolaCostFunctor::getPositionAtTime(x0, rotG, a, translation0, free0, collTime[0]);
//                ParabolaCostFunctor::getPositionAtTime(x1, rotG, a, translation1, free1, collTime[0]);
//                T dist = (Eigen::Map<Eigen::Matrix<T,3,1> >(x0)-Eigen::Map<Eigen::Matrix<T,3,1> >(x1)).norm();
            T dist = (Eigen::Map<const Eigen::Matrix<T,2,1> >(txy0)-Eigen::Map<const Eigen::Matrix<T,2,1> >(txy1)).norm();
            if (dist < _halfDiagSum) {
                residuals[0] = T(_sqrtWeight) * (dist - T(_halfDiagSum));
            }
            else
                residuals[0] = T(0.);

//                std::cout << "[" << __func__ << "] " << "diagonalhalf: " << _halfDiagSum << ", dist: " << dist << std::endl;
            return true;
        }
    protected:
        double _sqrtWeight;
        double _fx,_fy;
        double _t,_u,_v;
        double _halfDiagSum;
}; //...FitFunctor
#endif

struct ParabolaFitFunctor {
    public:
        using Scalar = ceres::CeresScalar;
        template <typename _Matrix3>
        ParabolaFitFunctor(_Matrix3 const& intr, Scalar t, Scalar u, Scalar v)
            : _fx( intr(0,0) ),
              _fy( intr(1,1) ),
              _t(t),
              _u((u - intr(0,2))/_fx),
              _v((v - intr(1,2))/_fy)
        {}

        enum { NUM_RESIDUALS = 2 };

        /** \param[in ] free Contains angle y0, linear b, scale s */
        template <typename T>
        bool operator()(T const rotG[2], T const a[1], T const txy[2], T const tz[1], T const free[3], T const collTime[1],
                        T* residuals) const {

            T const translation[3] = {txy[0], txy[1], tz[0]};
            T x[3];
            ParabolaCostFunctor::getPositionAtTime(x, rotG, a, translation, free, T(_t) - collTime[0]);
            if (x[2] == 0.)
                return false;

            residuals[0] = x[0] / x[2] - T(_u);
            residuals[1] = x[1] / x[2] - T(_v);
            return true;
        }

        template <typename _Matrix3>
        static ceres::CostFunction* Create(_Matrix3 const& intr, Scalar t, Scalar u, Scalar v) {
            return new ceres::AutoDiffCostFunction<ParabolaFitFunctor, ParabolaFitFunctor::NUM_RESIDUALS, 2, 1, 2, 1, 3, 1>(
                new ParabolaFitFunctor(intr, t, u, v));
        }
    protected:
        Scalar _fx,_fy;
        Scalar _t,_u,_v;
}; //...FitFunctor

struct DepthPriorFunctor {
    public:
        template <typename _Matrix3, typename _Vector3>
        DepthPriorFunctor(double sqrtWeight, _Matrix3 const& intr,double t, cv::Rect const& bbox, std::array<double,2> const& circleCentroid, float const circleRadius, _Vector3 const& objSize)
            : _sqrtWeight(sqrtWeight),
              _fx(intr(0,0)),
              _fy(intr(1,1)),
              _cx(intr(0,2)),
              _cy(intr(1,2)),
              _t(t),
              _bbox(bbox),
              _circleCentroid({circleCentroid[0],circleCentroid[1]}),
              _circleRadius(circleRadius),
              _diagonalLength(objSize.norm())
        {
            if (_diagonalLength < 0.1)
                std::cerr << "[" << __func__ << "] " << "adding sizeprior " << _diagonalLength << ", from " << objSize.transpose() << "...too small?" << std::endl;
        }

        enum { NUM_RESIDUALS = 1 };

        template <typename T>
        bool operator()(T const rotG[2], T const a[1], T const txy[2], T const tz[1], T const free[3], T const collTime[1],
                        T* residuals) const {

            T const translation[3] = {txy[0], txy[1], tz[0]};
            T x[3];
            ParabolaCostFunctor::getPositionAtTime(x, rotG, a, translation, free, T(_t) - collTime[0]);

            if (x[2] == 0.)
                return false;
#if 1
            Eigen::Matrix<T, 3, 1> p0(
                T((_bbox.tl().x - _cx) / _fx) * x[2],
                T((_bbox.tl().y - _cy) / _fy) * x[2],
                x[2]);

            Eigen::Matrix<T, 3, 1> p1(
                T((_bbox.br().x - _cx) / _fx) * x[2],
                T((_bbox.br().y - _cy) / _fy) * x[2],
                x[2]);
#else

            // a vector pointing from the circle centroid twoards the projection center
                Eigen::Matrix<T, 2, 1> v1(T(_circleCentroid[0] - _cx), T(_circleCentroid[1] - _cy));
                v1 = v1.normalized() * T(_circleRadius);

                Eigen::Matrix<T, 3, 1> p0(
                    T((_circleCentroid[0] - v1(0) - _cx) / _fx) * x[2],
                    T((_circleCentroid[1] - v1(0) - _cy) / _fy) * x[2],
                    x[2]);
                Eigen::Matrix<T, 3, 1> p1(
                    T((_circleCentroid[0] + v1(0) - _cx) / _fx) * x[2],
                    T((_circleCentroid[1] + v1(1) - _cy) / _fy) * x[2],
                    x[2]);
#endif

            residuals[0] = T(_sqrtWeight) * (T(_diagonalLength) - (p1 - p0).norm());

//                residuals[0] = x[0] / x[2] - T(_u);
//                residuals[1] = x[1] / x[2] - T(_v);
            return true;
        }
    protected:
        double const _sqrtWeight;
        double const _fx,_fy;
        double const _cx, _cy;
        double _t/*,_u,_v*/;
        cv::Rect const _bbox;
        std::array<double,2> _circleCentroid;
        double  const _circleRadius;
        double const _diagonalLength;
}; //...FitFunctor

#if 0
Eigen::Vector4d fit2dParabola(Track2D const& path) {
        ceres::Problem problem;
        Eigen::Vector4d coeffs( 1000.,-1.,0.1, 1.);
        for (std::pair<FrameId,TrackPoint2D> const& pair : path) {
            ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<Parabola2dFitFunctor, Parabola2dFitFunctor::NUM_RESIDUALS, 4>(new Parabola2dFitFunctor(pair.first,pair.second(0),pair.second(1)));
            problem.AddResidualBlock(costFunction, new ceres::HuberLoss(1.), coeffs.data());
        }
        problem.SetParameterLowerBound(coeffs.data(),2,0.);
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        ceres::Solver::Summary summary;
        ceres::Solve(options,&problem,&summary);
        std::cout << summary.BriefReport() << std::endl;
        return coeffs;
    } //...fit2dParabola
#endif

BundleWithPhysicsResult
fitParabola(Parabola2ds const& parabola2ds, FrameIdsT const& frameIds, Mapper const& mapper, Weights const& weights,
            CuboidsT const& cuboids, RgbsT const& rgbs, int const show, double* const cost, Scalar const collTimeArg,
            float const flatLimit, bool const saveRawCentroids) {
    using _Scalar = Cuboid::Scalar;
    const CollId collId(0);
    using ceres::CostFunction;

    ceres::Problem problem;
    PhysIndexer indexer(frameIds,std::count_if(cuboids.begin(), cuboids.end(), [](Cuboid const& cuboid) {
        return cuboid.isMassFinite(); }));
    indexer.allocate();

    for (auto const& entry : parabola2ds ) {
        CuboidId const& cuboidId = entry.first.first;
        if (!cuboids.at(cuboidId).isMassFinite())
            continue;
        PartId const& partId = entry.first.second;
        for (std::pair<FrameId,TrackPoint2D> const &pair : entry.second.getTrack() ) {
            FrameId const frameId = pair.first;
            // position
            if (show != 1 /*|| randf() < 0.3*/)
            {
                CostFunction *costFunction = ParabolaFitFunctor::Create(mapper.getIntrinsics(), frameId, pair.second(0),
                                                                       pair.second(1));
                //CostFunction *costFunction = new ceres::AutoDiffCostFunction<ParabolaFitFunctor, ParabolaFitFunctor::NUM_RESIDUALS, 2, 1, 2, 1, 3, 1>(
                //    new ParabolaFitFunctor(mapper.getIntrinsics(), frameId, pair.second(0), pair.second(1)));

                problem.AddResidualBlock(costFunction, new ceres::HuberLoss(.1), indexer.getParabolaRotationShared(),
                                         indexer.getParabolaSquaredParam(),
                                         indexer.getParabolaTranslation(cuboidId, collId),
                                         indexer.getParabolaTranslation(0, collId) + 2, // shared z coordinate
                                         indexer.getParabolaFreeParams(cuboidId, partId),
                                         indexer.getCollisionTime(collId));
            } //...position

            // depthPiror
            if (weights.sizePriorWeight > 0.) {
//                    std::cout << "[" << __func__ << "] areaWeight:" << weights.sizePriorWeight << std::endl;
                if (!entry.second.hasCircle(frameId)) {
                    std::cerr << "[" << __func__ << "] " << "no circle for frameId " << frameId << std::endl;
                }

                Circle const &circle = entry.second.getCircle(pair.first);
                ceres::CostFunction* costFunction = new ceres::AutoDiffCostFunction<DepthPriorFunctor, DepthPriorFunctor::NUM_RESIDUALS, 2, 1, 2, 1, 3, 1>(
                    new DepthPriorFunctor(weights.sizePriorWeight, mapper.getIntrinsics(), frameId,
                                          circle.getBoundingBox(), {circle.getCentroid().x, circle.getCentroid().y},
                                          circle.getRadius(), cuboids.at(cuboidId).getSize()));
                problem.AddResidualBlock(costFunction, new ceres::HuberLoss(.1), indexer.getParabolaRotationShared(),
                                         indexer.getParabolaSquaredParam(),
                                         indexer.getParabolaTranslation(cuboidId, collId),
                                         indexer.getParabolaTranslation(0, collId) + 2,
                                         indexer.getParabolaFreeParams(cuboidId, partId),
                                         indexer.getCollisionTime(collId));
            }//...depthPrior
        } //...for input circles
    } //...for parabolas

#if 0
    // repulsion
    if (false && weights.repulseWeight > 0.) {
        ceres::CostFunction *costFunction = new ceres::AutoDiffCostFunction<RepulseFunctor, RepulseFunctor::NUM_RESIDUALS, 2, 1, 2, 2, 1, 3, 3, 1>(
            new RepulseFunctor(weights.repulseWeight, mapper.getIntrinsics(), cuboids.at(0).getSize(), cuboids.size() > 1 ? &cuboids.at(1).getSize() : nullptr));
        problem.AddResidualBlock(costFunction, nullptr, indexer.getParabolaRotationShared(), indexer.getParabolaSquaredParam(),
                                 indexer.getParabolaTranslation(0, collId), indexer.getParabolaTranslation(1, collId),
                                 indexer.getParabolaTranslation(0, collId) + 2,
                                 indexer.getParabolaFreeParams(0, 0), indexer.getParabolaFreeParams(1, 0), indexer.getCollisionTime(collId));
    }
#endif

    // init
    if (collTimeArg > 0.) {
        indexer.getCollisionTime(collId)[0] = collTimeArg;
//            std::cout << "[" << __func__ << "] " << "limiting time to " << static_cast<int>(collTimeArg) << "and " << static_cast<int>(collTimeArg) + 1 << std::endl;
        if (weights.fixFlags & Weights::FIX_COLLTIMES)
            problem.SetParameterBlockConstant(indexer.getCollisionTime(collId));
        else {
            double lb = std::floor(collTimeArg);
            double ub = std::ceil(collTimeArg);
            if (ub - lb < kSmallDiff) {
                lb -= 1.;
                ub += 1.;
            }
            std::cout << "[" << __func__ << "] "
                      << "collTime limit:" << lb
                      << "<" << indexer.getCollisionTime(collId)[0]
                      << "<" << ub
                      << std::endl;

            problem.SetParameterLowerBound(indexer.getCollisionTime(collId), 0, lb);
            problem.SetParameterUpperBound(indexer.getCollisionTime(collId), 0, ub);
        }
    } else {
        indexer.getCollisionTime(collId)[0] = frameIds.at(collId + 1) + 0.5;
        problem.SetParameterBlockConstant(indexer.getCollisionTime(collId));
    }
    indexer.getParabolaRotationShared()[0] = 0.;
    indexer.getParabolaRotationShared()[1] = 0.;
//        std::cout << "fps: " << weights.fps << std::endl;
    initGravityTerm(problem,indexer,weights.fps);
    problem.SetParameterBlockConstant(indexer.getParabolaSquaredParam());
    problem.SetParameterBlockConstant(indexer.getParabolaRotationShared());
    for (auto const& entry : parabola2ds ) {
        CuboidId const &cuboidId = entry.first.first;
        if (!cuboids.at(cuboidId).isMassFinite())
            continue;
        PartId const& partId = entry.first.second;

        // y0
        indexer.getParabolaRotationFree(cuboidId, partId)[0] = 0.;
        //#warning Flatlimit commented out
        #if 1
        if (flatLimit > 0.) {
            problem.SetParameterLowerBound(indexer.getParabolaFreeParams(cuboidId,partId), 0, -flatLimit);
            problem.SetParameterUpperBound(indexer.getParabolaFreeParams(cuboidId,partId), 0,  flatLimit);
        } else {
            problem.SetParameterLowerBound(indexer.getParabolaFreeParams(cuboidId,partId), 0, -10.0);
            problem.SetParameterUpperBound(indexer.getParabolaFreeParams(cuboidId,partId), 0,  10.0);
        }
        #endif
        indexer.getParabolaBParam(cuboidId, partId)[0] = -0.05;
//#warning New constraint, b < 0.
//            std::cout << "[" << __func__ << "] limit: " << cuboidId << "," << partId << std::endl;
//            problem.SetParameterUpperBound(indexer.getParabolaFreeParams(cuboidId, partId),PhysIndexer::PARABOLA_FREE_B_OFFSET,0.);
        indexer.getParabolaSParam(cuboidId, partId)[0] = 1./weights.fps;
        indexer.getParabolaTranslation(cuboidId, collId)[0] = randf(0.1);
        indexer.getParabolaTranslation(cuboidId, collId)[1] = randf(0.1);
        indexer.getParabolaTranslation(cuboidId, collId)[2] = 1.;
        problem.SetParameterLowerBound(indexer.getParabolaTranslation(0, collId)+2,0,0.);
        problem.SetParameterUpperBound(indexer.getParabolaTranslation(0, collId)+2,0,15.);
    }

    ceres::Solver::Options options;
    if (weights.max_num_iterations > 0)
        options.max_num_iterations = weights.max_num_iterations;
    //options.linear_solver_type = ceres::DENSE_QR;
    ceres::Solver::Summary summary;
    ceres::Solve(options,&problem,&summary);
    std::cout << summary.BriefReport() << std::endl;
    if (summary.termination_type == ceres::FAILURE) {
        std::cerr << "[" << __func__ << "] " << "FAAAAAAAAAAAAAIL" << std::endl;
        std::cerr << "[" << __func__ << "] " << "FAAAAAAAAAAAAAIL" << std::endl;
        std::cerr << "[" << __func__ << "] " << "FAAAAAAAAAAAAAIL" << std::endl;
        std::cerr << "[" << __func__ << "] " << "FAAAAAAAAAAAAAIL" << std::endl;
        std::cerr << "[" << __func__ << "] " << "FAAAAAAAAAAAAAIL" << std::endl;
    }

    BundleWithPhysicsResult result;
    std::map<FrameId,cv::Mat> movie;
    ceres::CeresScalar const* rotG     = indexer.getParabolaRotationSharedConst();
    ceres::CeresScalar const* a        = indexer.getParabolaSquaredParamConst();
    ceres::CeresScalar const* collTime = indexer.getCollisionTimeConst(collId);
    if (show)
        std::cout << "collTime: " << collTime[0] << std::endl;
    result.a     = *a;
    ceres::CeresVector3 gravity;
    GravityCostFunctor::getWorldGravity(rotG,a,weights.fps,gravity.data());
    result.gravity = gravity.cast<Scalar>();
    result.rotX  = rotG[0];
    result.rotY1 = rotG[1];
    result.collTimes[collId] = collTime[0];

    result.tracks3d.reset(new GroupedTracks3d());
    GroupedTracks3d &tracks3d = *result.tracks3d;
    result.cuboids.reset(new CuboidsT());
    for (auto const& cuboid : cuboids) {
        result.cuboids->push_back(cuboid);
        if (cuboid.isMassFinite())
            result.cuboids->back().clearStates();
    }
    for (auto const& entry : parabola2ds ) {
        CuboidId const &cuboidId = entry.first.first;
        if (!cuboids.at(cuboidId).isMassFinite())
            continue;

        PartId const &partId   = entry.first.second;
        if (cuboidId > 1) {
            std::cerr << "[" << __func__ << "] " << "Cuboid id unexepectedly larger than 1..." << std::endl;
            throw new std::runtime_error("");
        }

        if (tracks3d.getIndices().find(cuboidId) == tracks3d.getIndices().end())
            tracks3d.addTrack(Track3D(cuboidId),cuboidId);
        Track3D &track3D = tracks3d.getTrackByLabel(cuboidId);

        ceres::MapConstCeresVector3 freeParams   (indexer.getParabolaFreeParamsConst(cuboidId,partId));
        ceres::MapConstCeresVector3 translationXy(indexer.getParabolaTranslationConst(cuboidId,collId));
        ceres::CeresScalar const* const translationZ = indexer.getParabolaTranslationConst(0,collId)+2;
        ceres::CeresVector3 translation(translationXy(0), translationXy(1), *translationZ);

        double niceAngle = freeParams[0];
        while (niceAngle < -180.) niceAngle += 360.;
        while (niceAngle > 180.) niceAngle -= 360.;
        if (show) {
            std::cout << "[" << __func__ << "] parabola[" << cuboidId << "," << partId << "]\n\ttranslation: "
                      << translation.transpose()
                      << "\n\ty_0: " << freeParams[0] << "(" << niceAngle << ") b: " << freeParams[1] << ", s: "
                      << freeParams[2] << std::endl;
        }
        Eigen::Vector3d x;
        Cuboid &cuboid = result.cuboids->at(cuboidId);
        for (FrameId frameId = frameIds.front(); frameId <= frameIds.back(); step(frameId)) {
            if (partId != getPartId(frameId,frameIds))
                continue;

            ParabolaCostFunctor::getPositionAtTime(x.data(), rotG, a, translation.data(), freeParams.data(), frameId - collTime[0]);
            track3D.addPoint(frameId, TrackPoint3D::WithNoNormal(x.cast<Scalar>()));

            if (show && movie.find(frameId) == movie.end())
                movie[frameId] = rgbs.at(frameId).clone();
            if (saveRawCentroids) {
                if (entry.second.hasCircle(frameId)) {
                    auto const p2 = entry.second.getCircle(frameId).getCentroid();
                    cuboid.setPosition(frameId,mapper.to3D(Eigen::Vector2f{p2.x,p2.y}, x[2]));
//                        cuboid.setPosition(frameId, {p2.x, p2.y, -1.f});
                }
            } else
                cuboid.setPosition(frameId,x.cast<_Scalar>());
        }
//            result.cuboids->emplace_back(std::move(cuboid));
        Parabola parabola {*indexer.getParabolaBParam(cuboidId,partId),
                           *indexer.getParabolaSParam(cuboidId,partId),
                           *indexer.getParabolaRotationFree(cuboidId,partId),
                           translation.data()};
        result.parabolas[cuboidId][partId] = std::move(parabola);
    }

    if (show) {
        static std::map<TrackId,cv::Scalar> colors;
        for (auto const &track : tracks3d) {
            if (colors.find(track.getTrackId()) == colors.end()) {
                auto const tmpColor = colors::paletteMediumColoursEigen2(colors.size()+1,255.).at(colors.size());
                colors[track.getTrackId()] = cv::Scalar { tmpColor(0), tmpColor(1), tmpColor(2) };
            }
            cv::Scalar const& color = colors.at(track.getTrackId());
            std::list<cv::Point2i> history;
            for (auto &frameIdAndImg : movie) {
                FrameId const &frameId = frameIdAndImg.first;
                cv::Mat       &img     = frameIdAndImg.second;

                if (!track.hasPoint(frameId))
                    continue;
                TrackPoint3D const &point3D = track.getPoint(frameId);
                auto p2 = mapper.to2D(point3D.getPoint());

                cv::Point2i p1 {static_cast<int>(std::round(p2(0))), static_cast<int>(std::round(p2(1)))};
                cv::circle(img, p1, 1, color);
                history.push_back(p1);
                if (history.size() > 1) {
                    auto it0 = history.begin();
                    for (auto it1 = std::next(history.begin()); it1 != history.end(); ++it0, ++it1)
                        cv::line(img, *it0, *it1, color, 3);
                }
            }
        }

//            for (auto const& entry : parabola2ds ) {
//                CuboidId const& cuboidId = entry.first.first;
//                cv::Scalar const& color = colors.at(cuboidId);
//                for (std::pair<FrameId,TrackPoint2D> const &pair : entry.second.getTrack() ) {
//                    cv::circle(movie.at(pair.first), cv::Point2i(pair.second(0), pair.second(1)),5,color);
//                }
//            }

        io::my_mkdir("parabola2d");
        char name[255];
//        for (auto iter = std::begin(movie); iter != std::end(movie); ++iter) {
        auto iter = movie.rbegin(); {
            for (auto const& entry : parabola2ds ) {
                CuboidId const& cuboidId = entry.first.first;
                PartId   const& partId   = entry.first.second;
                cv::Scalar color = colors.at(cuboidId);
                //if (partId) color = color * 0.7 + 0.3 * cv::Scalar(.1*255.,.1*255.,.8*255.);
                //else        color = color * 0.7 + 0.3 * cv::Scalar(.8*255.,.1*255.,.1*255.);
                for (std::pair<FrameId,TrackPoint2D> const &pair : entry.second.getTrack() ) {
                    FrameId const frameId = pair.first;
                    cv::circle(iter->second, cv::Point2i(pair.second(0), pair.second(1)), partId ? 5 : 2, color);
                    auto const p3d = tracks3d.getTrackByLabel(cuboidId).getPoint(frameId).getPos();
                    auto const p2d = mapper.to2D(p3d);
                    cv::line(iter->second,
                             cv::Point2i(pair.second(0), pair.second(1)),
                             cv::Point2i(p2d(0), p2d(1)),
                             cv::Scalar(0.,0.,0.));
                }
            }
            sprintf(name, "p2_%05u.jpg", iter->first);
            cv::imshow("2dparabola" + std::to_string(show), iter->second);
            cv::putText(iter->second, std::to_string(show), cv::Point2i(0,0), 1, 3, cv::Scalar(255.,255.,255));
            cv::imwrite(std::string("parabola2d/") + name, iter->second);
            char c = 0; while ((c=cv::waitKey())!=27);
        }
        //char c(0);
        //while ((c = cv::waitKey()) != 27);
    } //...if show

    // make sure collTime is not hitting a limit
    if (std::abs(static_cast<float>(frameIds.at(1)) - result.collTimes.at(0)) < kSmallDiff) {
        std::cout << "[" << __func__ << "] " << "collTime: " << collTime[0] << result.collTimes.at(0) << ", frameIds.at1: " << frameIds.at(1) << " +0.1" << std::endl;
        result.collTimes.at(0) = static_cast<float>(frameIds.at(1)) + 0.1;
    } else if (std::abs(static_cast<float>(getNext(frameIds.at(1))) - result.collTimes.at(0)) < kSmallDiff) {
        std::cout << "[" << __func__ << "] " << "collTime: " << collTime[0] << ", collTimes.at(0): " << result.collTimes.at(0) << ", frameIds.at1: " << frameIds.at(1) << " -0.1" << std::endl;
        result.collTimes.at(0) = getNext(frameIds.at(1)) - 0.1;
    }
    std::cout << "[" << __func__ << "] " << "corrected colltime: " << result.collTimes.at(0) << std::endl;
    result.setCircles(parabola2ds);

    if (cost)
        *cost = summary.final_cost;
    return result;
} //...fit2dParabola

} //...ns bundle_physics
} //...ns tracking
