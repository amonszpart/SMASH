//
// Created by bontius on 10/01/16.
//

//#include "tracking/phys/physFunctors.h"
#include <bitset>
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/vis/visPhys.h"
#include "tracking/phys/vis/vis.h"
#include "tracking/phys/consts.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/annot/annotator.h"
#include "tracking/common/io/os.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/tracks.h"
#include "tracking/common/mapper.h"
#include "tracking/common/util/colors.h"
#include "ceres/ceresUtil.h"

namespace tracking {
  namespace bundle_physics {

    DEFINE_EXCEPTION(ShowPhysOutput_MassMismatch)
    // BundleWithPhysicsResult &out, const CuboidsT &cuboids, const Tracks2D &tracks2d, const FrameIdsT &frameIds, const PhysIndexer &indexer, const Consts &consts
    void showPhysOutput(const BundleWithPhysicsResult& out, const CuboidsT& cuboids, const PhysIndexer& indexer, const Consts &consts, std::string const showFlags = "111111", FrameIdsT const* const frameIds) {
        using ceres::MapConstCeresVector3;
        using ceres::CeresVector3;
        using ceres::CeresScalar;
        //typedef Cuboid::Scalar       Scalar;
        //typedef Cuboid::Vector3      Vector3;
        //typedef Cuboid::TranslationT TranslationT;
        enum {SHOW_IN_CENTROIDS = 0, SHOW_IN_CUB = 1, SHOW_OUT_CENTROIDS = 2, SHOW_OUT_POSES = 3, SHOW_COLL_NORMAL = 4};
        std::bitset<6> show {showFlags};
        std::cout << "[" << __func__ << "] "
                  << "flags: " << showFlags << ": "
                  << "SHOW_IN_CENTROIDS: " << show[SHOW_IN_CENTROIDS]
                  << ", SHOW_IN_CUB: " << show[SHOW_IN_CUB]
                  << ", SHOW_OUT_CENTROIDS: " << show[SHOW_OUT_CENTROIDS]
                  << ", SHOW_OUT_POSES: " << show[SHOW_OUT_POSES]
                  << ", SHOW_COLL_NORMAL: " << show[SHOW_COLL_NORMAL]
                  << std::endl;

        using namespace Soup::vis;
        char name[512];
        const Scalar sceneScale = 1.;
//        const CeresScalar *const rotationShared(indexer.getParabolaRotationSharedConst());

        Visualizer <Scalar> vis("Initialization"), v2("Optimized");
//        vis.addCoordinateSystem(sceneScale);
//        v2.addCoordinateSystem(sceneScale);
        for (CuboidId cuboidId = 0; cuboidId != static_cast<CuboidId>(cuboids.size()); ++cuboidId) {
            // input
            const Cuboid &cuboid = cuboids.at(cuboidId);
            const Cuboid &outCuboid = out.cuboids->at(cuboidId);
            if (show[SHOW_IN_CUB] || show[SHOW_IN_CENTROIDS]) {
                drawCuboidStates2(cuboid, vis, sceneScale, /* color: */ Vector3(0., .3, 0.), cuboidId, frameIds, show[SHOW_IN_CENTROIDS]);
                drawCuboidStates2(cuboid, v2, sceneScale, /* color: */ Vector3(0., .3, 0.), cuboidId, frameIds, show[SHOW_IN_CENTROIDS]);
            }
            CeresScalar const* const mass = indexer.getMassConst(cuboidId);

            CeresVector3 invI;
            cuboid.getInverseIFromMass(invI, outCuboid.getMass() );
            std::cout << "mass: " << mass[0] << ", invI " << invI.transpose() << std::endl;
            if ( std::abs(mass[0]-outCuboid.getMass()) > kSmallDiff )
            {
                std::cerr << "mass mismatch!" << mass[0] << " vs. " << outCuboid.getMass() << std::endl;
                throw new ShowPhysOutput_MassMismatchException("");
            }

            PoseLoc const* prevPoseLoc(nullptr);
            for (const Cuboid::StatesT::value_type &frameIdAndState : out.cuboids->at(cuboidId).getStates()) {
                const FrameId frameId  = frameIdAndState.first;
                const PoseLoc &poseLoc = frameIdAndState.second;
                if (frameIds && (frameId < frameIds->front() || frameId > frameIds->back()))
                    continue;

                // dx
                if (prevPoseLoc) {
                    sprintf(name, "cuboid%d_frameId%u", cuboidId, frameId);
//                    vis.addArrow(sceneScale * prevPoseLoc->getPosition(), sceneScale * poseLoc.getPosition(), Vector3(.8, 0., 0.), name, 0.2);
                    if (show[SHOW_OUT_CENTROIDS])
                        v2.addSphere(sceneScale * prevPoseLoc->getPosition(), 0.025, Vector3(.8, 0., 0.), name);
                }
                prevPoseLoc = &poseLoc;

                // pose
                if (!(frameId % (consts.substeps*4)) /*&& cuboids.at(cuboidId).hasFrame(frameId / consts.substeps)*/) {
                    sprintf(name, "optCuboid%d_time%u", cuboidId, frameId);
                    if (show[SHOW_OUT_POSES])
                        drawCuboid(v2, TranslationT(sceneScale * poseLoc.getPosition()) * poseLoc.getPose().toRotationMatrix() * cuboid.getSizeTransform(), Vector3(.3, 0., 0.), name);
                }

                // vel
                //sprintf( name, "vel_cub%d_time%u", cuboidId, frameId );
                //Vector3 vel = poseLoc.getLinVel();
                //vis.addArrow( (sceneScale * poseLoc.getPosition()).cast<Scalar>(), (sceneScale * poseLoc.getPosition()).cast<Scalar>() + vel/2., Vector3::Zero(), name );
            } //...states

//            for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
//                sprintf(name, "translation_cub%d_coll%d", cuboidId, collId);
//                MapConstCeresVector3 translation(indexer.getParabolaTranslationConst(cuboidId, collId));
//                std::cout << "translation: " << translation.transpose() << std::endl;
                //vis.addArrow( Vector3::Zero(), sceneScale * translation.cast<Scalar>(), Vector3::Zero(), name );
//                const CeresScalar *const rotationFree0(indexer.getParabolaRotationFreeConst(cuboidId, collId));
//                const CeresScalar *const rotationFree1(indexer.getParabolaRotationFreeConst(cuboidId, collId + 1));
//                const CeresScalar *const b0(indexer.getParabolaBParamConst(cuboidId, collId));
//                const CeresScalar *const b1(indexer.getParabolaBParamConst(cuboidId, collId + 1));
//                const CeresScalar *const s0(indexer.getParabolaSParamConst(cuboidId, collId));
//                const CeresScalar *const s1(indexer.getParabolaSParamConst(cuboidId, collId + 1));
//                std::cout << "rotation0: " << rotationFree0[0] << "," << rotationShared[0] << "," << rotationShared[1] << std::endl;
//                std::cout << "rotation1: " << rotationFree1[0] << "," << rotationShared[0] << "," << rotationShared[1] << std::endl;
//                std::cout << "sba0: " << s0[0] << "," << b0[0] << "," << rotationShared[2] << std::endl;
//                std::cout << "sba1: " << s1[0] << "," << b1[0] << "," << rotationShared[2] << std::endl;
//            }
        } //...for all cuboids

        for (CollId collId = consts.firstCollId; collId <= consts.lastCollId; ++collId) {
            MapConstCeresVector3 collPoint(indexer.getCollisionPointConst(collId)); // relative to centroidA
            auto                 posA0 = out.collisions.at(collId).states.first.at(0).getPosition();
            //auto                 posA1 = out.collisions.at(collId).states.second.at(0).getPosition();
//            vis.addSphere(sceneScale * posA0, 0.01, Vector3(1., 0., .0), "posA0");
//            vis.addSphere(sceneScale * posA1, 0.01, Vector3(.5, .5, 1.), "posA1");
            if (out.collisions.at(collId).states.first.size() > 1) {
                //auto posB0 = out.collisions.at(collId).states.first.at(1).getPosition();
//                vis.addSphere(sceneScale * posB0, 0.01, Vector3(0., .0, 1.), "posB0");
                //auto posB1 = out.collisions.at(collId).states.second.at(1).getPosition();
//                vis.addSphere(sceneScale * posB1, 0.01, Vector3(1., .5, .5), "posB1");
            }

            if (show[SHOW_COLL_NORMAL]) {
                auto const startPnt = sceneScale * (posA0 + collPoint.cast<Scalar>());
                vis.addSphere(startPnt, 0.01, Vector3(.8, .8, .5), "collPoint");
                vis.addArrow(startPnt, startPnt + out.collisions.at(0).impulse * 2., Vector3::Zero(), "impulse");
                std::cout << "impulse: " << out.collisions.at(0).impulse.transpose() << std::endl;
                std::cout << "collPoint: " << collPoint.transpose() << std::endl;
            }
        } //...for collisions
        vis.spin();
    } //...showPhysOutput()

    void createMovie( CuboidsT const& cuboids, tracking::LinRgbsT const& rgbs, const FrameIdsT& frameIds, const Mapper& mapper, const CuboidsT* const inCuboids,
                      Tracks3D const* const tracks3d, const Tracks2D* const tracks2d, Vector3 const* const gravity, int const fps) {
        const int substeps = 3;
        ::io::my_mkdir( "tmp", 755 );
        cv::Mat opt(rgbs.at(0).clone()), in(rgbs.at(0).clone());
        std::vector<cv::Mat> movie;
        std::vector<cv::Scalar> colors;
        if (tracks3d)
            colors = colors::nColoursCv<cv::Scalar>( tracks3d->size(), 255., true );

        FrameId linId = 0;
        for (FrameId frameId = frameIds.front() * substeps; frameId <= frameIds.back() * substeps; step(frameId), step(linId)) {
            //std::cout << "[" << __func__ << "] " << "fetching " << linId / substeps << " / " << rgbs.size() << std::endl;
            cv::Mat const& rgb = rgbs.at(linId / substeps);
            cv::Rect const left (       0, 0,rgb.cols,rgb.rows);
            cv::Rect const right(rgb.cols, 0,rgb.cols,rgb.rows);
            while ( movie.size() <= linId )
            {
                movie.push_back( cv::Mat::zeros(rgb.rows, rgb.cols * 2, CV_8UC3) );
                rgb.copyTo( movie.back()( left ) );
                rgb.copyTo( movie.back()( right ) );
            }
            cv::Mat movieFrame = movie.at( linId );

            // tracks
            if (tracks3d) {
                cv::Mat rightFrame = movieFrame(right);
                cv::Mat leftFrame = movieFrame(left);
                int trackId(0);
                for (Track3D const& track3d : *tracks3d ) {
                    if (track3d.hasPoint(frameId)) {
                        Mapper::Vector2 p2 = mapper.to2D( track3d.getPoint(frameId).getPoint() );
                        cv::Point2i cvPoint2( p2.coeff(0), p2.coeff(1) );
                        if (cvPoint2.x < right.width && cvPoint2.y < right.height && cvPoint2.x >= 0 && cvPoint2.y >= 0) {
                            cv::circle( rightFrame, cvPoint2, 1, colors.at(trackId % colors.size()), 1 );
                        }

                        if (tracks2d && tracks2d->hasLabel(track3d.getLabel())) {
                            Track2D const& track2d = tracks2d->getTrackByLabel( track3d.getLabel() );
                            if ( track2d.hasPoint(frameId/substeps) ) {
                                cv::Point2i cvPoint2In( track2d.getPoint(frameId/substeps)[0], track2d.getPoint(frameId/substeps)[1] );
                                if (cvPoint2In.x < right.width && cvPoint2In.y < right.height && cvPoint2In.x >= 0 && cvPoint2In.y >= 0) {
                                    cv::circle(rightFrame, cvPoint2In, 1, cv::Scalar(0.,0.,0.), 1 );
                                    cv::line  (rightFrame, cvPoint2, cvPoint2In, cv::Scalar(0.,0.,0.) );
                                }

                                cv::circle( leftFrame, cvPoint2In, 1, colors.at(trackId % colors.size()), 1 );
                            }
                        }
                    }
                    ++trackId;
                }
            } //...if tracks3d

            int cuboidId(0);
            for (auto const &cuboid : cuboids) {
                //const FrameId inFrameId = frameId / substeps;
                if (cuboid.hasFrame(frameId)) {
                    cv::Mat rightFrame(movieFrame(right));
                    auto transform = cuboid.getTransformation(frameId);

                    Eigen::Matrix3f cs(Eigen::Matrix3f::Identity() * 0.3);
                    cs = transform * cs;
                    auto const pos = mapper.to2D(transform.translation());
                    for (int   i   = 0; i != 3; ++i) {
                        auto const dir = mapper.to2D(cs.col(i));
                        cv::line(rightFrame, cv::Point2i(pos(0), pos(1)), cv::Point2i(dir(0), dir(1)), cv::Scalar(i == 0 ? 255. : 0., i == 1 ? 255. : 0., i == 2 ? 255. : 0.), 2);
                    }

                    Annotator::drawCuboid2(movieFrame(right), transform, mapper);

                    if (gravity) {
                        auto centroid3 = transform.translation();
                        auto centroid2 = mapper.to2D( centroid3 );
                        cv::putText(rightFrame, std::to_string(cuboidId), cv::Point2i(centroid2(0),centroid2(1)), 3, 3, cv::Scalar(128,200,128));
                        auto gravity2 = mapper.to2D( centroid3 + (*gravity * 0.01) );
                        cv::line( rightFrame,
                            cv::Point2i(centroid2(0),centroid2(1)),
                            cv::Point2i(gravity2(0) ,gravity2(1)),
                            cv::Scalar(243,170,216), 2 );
                    }
                } //...hasFrame

                // input
                FrameId inFrameId = frameId/substeps;
                if (inCuboids && (cuboidId < CuboidId(inCuboids->size())) && inCuboids->at(cuboidId).hasFrame(inFrameId)) {
                    if (!inCuboids->at(cuboidId).getState(inFrameId).hasPos())
                       continue;
                    if (inCuboids->at(cuboidId).getState(inFrameId).hasPose() )
                        Annotator::drawCuboid2(movieFrame(left), inCuboids->at(cuboidId).getTransformation(inFrameId), mapper );
                    else if (inCuboids->at(cuboidId).getState(inFrameId).hasPos()) {
                        auto const pnt = mapper.to2D(inCuboids->at(cuboidId).getState(inFrameId).getPosition());
                        cv::circle(movieFrame(left), cv::Point2i(pnt(0),pnt(1)), 3, cv::Scalar(200.,10.,10.),-1);
                    }
                }

                ++cuboidId;
            } //...for cuboids

            // frameName
            {
                char frameName[255];
                sprintf(frameName, "%02u", frameId);
                cv::Mat leftPart = movieFrame(left);
                cv::putText(leftPart, frameName, cv::Point2i(20, 20), 1, 2., cv::Scalar(255., 255., 255.));
            }
        } // ...for frames

        int i(0), j(0);
        char path[2048];
        std::vector<int> imWriteParams = {cv::IMWRITE_PNG_COMPRESSION, 1};
        for ( auto const& movieFrame : movie )
        {
            if (fps < 240 || !(i % substeps)) {
                sprintf( path, "tmp/opt_%05d.png", j++ );
                cv::imwrite( path, movieFrame, imWriteParams );
                cv::imshow("movie", movieFrame);
                cv::waitKey(10);
            }
            ++i;
        }

    } //...createMovie()
  } //...ns bundle_physics
} //...ns tracking