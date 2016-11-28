// Ceres Solver - A fast non-linear least squares minimizer
// Copyright 2015 Google Inc. All rights reserved.
// http://ceres-solver.org/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice,
//   this list of conditions and the following disclaimer.
// * Redistributions in binary form must reproduce the above copyright notice,
//   this list of conditions and the following disclaimer in the documentation
//   and/or other materials provided with the distribution.
// * Neither the name of Google Inc. nor the names of its contributors may be
//   used to endorse or promote products derived from this software without
//   specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Author: keir@google.com (Keir Mierle)
//
// A minimal, self-contained bundle adjuster using Ceres, that reads
// files from University of Washington' Bundle Adjustment in the Large dataset:
// http://grail.cs.washington.edu/projects/bal
//
// This does not use the best configuration for solving; see the more involved
// bundle_adjuster.cc file for details.

#include "tracking/common/track.h"
#include "tracking/annot/bundleAdjuster.h"
#include "tracking/vis/visualizer.h"
#include "ceres/functorInfo.h"
#include "ceres/ceresUtil.h"
#include "ceres/rotation.h"
#include "ceres/ceres.h"
#include "opencv2/highgui/highgui.hpp"

#include <cmath>
#include <cstdio>
#include <iostream>

// Templated pinhole camera model for used with Ceres.  The camera is
// parameterized using 9 parameters: 3 for rotation, 3 for translation, 1 for
// focal length and 2 for radial distortion. The principal point is not modeled
// (i.e. it is assumed be located at the image center).
struct SnavelyReprojectionError {
        SnavelyReprojectionError(double observed_x, double observed_y, double focalX, double focalY)
            : observed_x(observed_x), observed_y(observed_y), _focalX( focalX ), _focalY( focalY ) {}

        template <typename T, typename T2>
        static void to2d( T& predicted_x, T& predicted_y, const T* p, const T2& focalX, const T2& focalY )
        {
            T xp = - p[0] / p[2];
            T yp = - p[1] / p[2];
            predicted_x = focalX * xp;
            predicted_y = focalY * yp;
        }

        template <typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        const T* const /*intrinsics*/, // fx,fy,cx,cy,l1,l2
                        T* residuals) const {
            // camera[0,1,2] are the angle-axis rotation.
            T p[3];
            ceres::AngleAxisRotatePoint(camera, point, p);

            // camera[3,4,5] are the translation.
            p[0] += camera[3];
            p[1] += camera[4];
            p[2] += camera[5];
#if 1
            T predicted_x, predicted_y;
            to2d( predicted_x, predicted_y, p, _focalX, _focalY );
#else
            // Compute the center of distortion. The sign change comes from
            // the camera model that Noah Snavely's Bundler assumes, whereby
            // the camera coordinate system has a negative z axis.
            T xp = - p[0] / p[2];
            T yp = - p[1] / p[2];

            // Apply second and fourth order radial distortion.
//            const T& l1 = intrinsics[4];
//            const T& l2 = intrinsics[5];
//            T r2 = xp*xp + yp*yp;
//            T distortion = T(1.0) + r2  * (l1 + l2  * r2);

            // Compute final projected point position.
            static const T focalx = T(1080.89),// intrinsics[0],
                           focaly = T(912.);//intrinsics[1],
//                           cx     = T(640.),//intrinsics[2],
//                           cy     = T(360.);// intrinsics[3];
//            const T &focalx = intrinsics[0],
//                    &focaly = intrinsics[1];
//                    &cx     = intrinsics[2],
//                    &cy     = intrinsics[3];

            T predicted_x = focalx * xp;
            T predicted_y = focaly * yp;
#endif

            //xy[0] = (  uvz[0] / T(intrinsics(0,0)) - T(intrinsics(0,2) / intrinsics(0,0))  ) * uvz[2];
            //xy[1] = (  uvz[1] / T(intrinsics(1,1)) - T(intrinsics(1,2) / intrinsics(1,1))  ) * uvz[2];
            // u = (x/z + cx/fx) * fx = fx * x / z + cx;
            // v = (y/z + cy/fy) * fy = fy * x / z + cy;

            // The error is the difference between the predicted and observed position.
            residuals[0] = predicted_x - T(observed_x);
            residuals[1] = predicted_y - T(observed_y);
//            residuals[2] = ceres::abs(camera[3]) > 10. ? camera[3] : T(0.);
//            residuals[3] = ceres::abs(camera[4]) > 10. ? camera[4] : T(0.);
//            residuals[4] = ceres::abs(camera[5]) > 10. ? camera[5] : T(0.);
//            residuals[5] = ceres::abs(p[0]) > 10. ? p[0] : T(0.);
//            residuals[6] = ceres::abs(p[1]) > 10. ? p[1] : T(0.);
//            residuals[7] = ceres::abs(p[2]) > 10. ? p[2] : T(0.);
            //std::cout << "pred: " << predicted_x << "," << predicted_y << " vs. " << observed_x << "," << observed_y << std::endl;

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double observed_x,
                                           const double observed_y, const double focalX, const double focalY)
        {
            return (new ceres::AutoDiffCostFunction<SnavelyReprojectionError, 2, 6, 3, 4>(
                        new SnavelyReprojectionError(observed_x, observed_y, focalX, focalY)));
        }
    protected:
        double observed_x;
        double observed_y;
        double _focalX, _focalY;
};

struct CameraSmoothnessError {
        CameraSmoothnessError( double weight )
            : _weight(weight) {}

        enum { NUM_RESIDUALS = 3 };

        template <typename T>
        bool operator()(const T* const camera0,
                        const T* const camera1,
                        T* residuals) const
        {
            residuals[0] = T(_weight) * (camera0[3] - camera1[3]);
            residuals[1] = T(_weight) * (camera0[4] - camera1[4]);
            residuals[2] = T(_weight) * (camera0[5] - camera1[5]);

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double weight) {
            return (new ceres::AutoDiffCostFunction<CameraSmoothnessError, NUM_RESIDUALS, 6, 6>(
                        new CameraSmoothnessError(weight)));
        }
    protected:
        const double _weight;

};

struct PointSmoothnessError {
        PointSmoothnessError( double weight )
            : _weight(weight) {}

        enum { NUM_RESIDUALS = 3 };

        template <typename T>
        bool operator()(
                        const T* const camera0,
                        const T* const camera1,
                        const T* const point,
                        T* residuals) const
        {
            T p0[3], p1[3];
            ceres::AngleAxisRotatePoint(camera0, point, p0);
            ceres::AngleAxisRotatePoint(camera1, point, p1);

            residuals[0] = T(_weight) * (p0[0] - p1[0]);
            residuals[1] = T(_weight) * (p0[1] - p1[1]);
            residuals[2] = T(_weight) * (p0[2] - p1[2]);

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double weight) {
            return (new ceres::AutoDiffCostFunction<PointSmoothnessError, NUM_RESIDUALS, 6, 6, 3>(
                        new PointSmoothnessError(weight)));
        }
    protected:
        const double _weight;

};

namespace tracking {
int BundleAdjuster::main(       tracking::Tracks3D& tracks3d,
                                TransformsT       & outTransforms,
                          const tracking::Tracks2D& tracks2d,
                          const Mapper            & mapper,
                          const FrameId             startFrame,
                          const FrameId             endFrame, // inclusive!
                          const OptParams& optParams,
                          const std::vector< ::Eigen::Vector3f >& colors,
                          const std::vector< cv::Mat >          & rgbs,
                                TransformsT       * inTransforms
                          )
{
    using namespace tracking;
    using _Scalar = tracking::bundle_physics::Scalar;
    tracks3d.clear();

    //  google::InitGoogleLogging( argv[0] );

    //  BALProblem bal_problem;
    //  if (!bal_problem.LoadFile(argv[1])) {
    //    std::cerr << "ERROR: unable to open file " << argv[1] << "\n";
    //    return 1;
    //  }

    //const double* observations = bal_problem.observations();
    const int nFrames = endFrame - startFrame + 1;
    std::vector<double> cameras( nFrames * CAMERA_STRIDE, 1. );
    for ( size_t i = 0; i != cameras.size(); ++i )
        cameras[i] = rand()/static_cast<float>(RAND_MAX) * 2.;
    std::vector<double> points ( tracks2d.size() * POINT_STRIDE, 1. );
    std::vector<double> intrinsics( 6, 0. ); // fx, fy, cx, cy, l1, l2
    intrinsics[0] = mapper.getIntrinsics()(0,0); // fx = 1080.89
    intrinsics[1] = mapper.getIntrinsics()(1,1); // fy = 912.
    intrinsics[2] = mapper.getIntrinsics()(0,2); // cx = 640.
    intrinsics[3] = mapper.getIntrinsics()(1,2); // cy = 360.
    Indexer indexer( startFrame, tracks2d.size() );

    std::vector< ceres::FunctorInfo >  costFunctors;
    char name[512];

    // Create residuals for each observation in the bundle adjustment problem. The
    // parameters for cameras and points are added automatically.
    ceres::LossFunction* loss = new ceres::HuberLoss(100.);
    ceres::Problem       problem;
    {
        TrackId              trackId( 0 ); // keep count
        // for each 2D obsevation over time
        for ( const Track2D& track : tracks2d )
        {
            CeresScalar* pointAddress = indexer.getPoint ( points.data(), trackId );
            // for each time point
            for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId )
            {
                CeresScalar* cameraAddress = indexer.getCamera( cameras.data(), frameId );

                // if the point was observed at this time point
                if ( track.hasPoint(frameId) )
                {
                    // cache
                    const TrackPoint2D &point2d = track.getPoint( frameId );

                    // create functor with observation
                    ceres::CostFunction* cost_function =
                            SnavelyReprojectionError::Create( point2d(0),
                                                              point2d(1), mapper.getIntrinsics()(0,0), mapper.getIntrinsics()(1,1) );
                    // store 3D point variable's address
                    problem.AddResidualBlock(cost_function,
                                             NULL,
                                             cameraAddress,
                                             pointAddress,
                                             intrinsics.data()
                                             );
                    sprintf( name, "reprojError_track%u_time%u", trackId, static_cast<unsigned>(frameId) );
                    costFunctors.emplace_back( ceres::FunctorInfo(name, cost_function, {cameraAddress,pointAddress,intrinsics.data()}) );

                    // init
                    pointAddress[0] = point2d(0);
                    pointAddress[1] = point2d(1);
                    pointAddress[2] = 1. + rand()/static_cast<float>(RAND_MAX) * 2.;
                    std::cout << "init[" << frameId << "][" << trackId << "]: " << ceres::MapCeresVector3(pointAddress).transpose() << std::endl;
                } //...if track has frame

                // camera smoothness
//                    if ( frameId != endFrame )
//                    {
//                        ceres::CostFunction* cost_function = CameraSmoothnessError::Create( .0001 );
//                        // store 3D point variable's address
//                        CeresScalar* nextCameraAddress = indexer.getCamera( cameras.data(), getNext(frameId) );
//                        problem.AddResidualBlock(cost_function,
//                                                 loss,
//                                                 cameraAddress,
//                                                 nextCameraAddress
//                                                 );
//                        sprintf( name, "CameraSmoothError_time%u",  static_cast<unsigned>(frameId) );
//                        costFunctors.emplace_back( ceres::FunctorInfo(name, cost_function, {cameraAddress,nextCameraAddress,pointAddress}) );
//                    }

                // point smoothness
                if ( frameId != endFrame )
                {
                    ceres::CostFunction* cost_function = PointSmoothnessError::Create( .01 );
                    // store 3D point variable's address
                    CeresScalar* nextCameraAddress = indexer.getCamera( cameras.data(), getNext(frameId) );
                    problem.AddResidualBlock(cost_function,
                                             loss,
                                             cameraAddress,
                                             nextCameraAddress,
                                             pointAddress
                                             );
                    sprintf( name, "PointSmoothError_time%u",  static_cast<unsigned>(frameId) );
                    costFunctors.emplace_back( ceres::FunctorInfo(name, cost_function, {cameraAddress,nextCameraAddress,pointAddress}) );
                }

                if ( inTransforms )
                {
                    if ( inTransforms->find(frameId) != inTransforms->end() )
                    {
                        std::cout << "initing camera of frame " << frameId << ", with\n" << inTransforms->at(frameId).matrix() << std::endl;
                        Eigen::Matrix3d rot = inTransforms->at(frameId).rotation().cast<CeresScalar>();
                        ceres::RotationMatrixToAngleAxis( ceres::MatrixAdapter<const CeresScalar,Matrix3d::OuterStrideAtCompileTime,Matrix3d::InnerStrideAtCompileTime>(rot.data()),
                                                          cameraAddress );
                        (ceres::MapCeresVector3(cameraAddress+3)) = inTransforms->at(frameId).translation().cast<CeresScalar>();
                    }
                }
            } //...for each frame

            ++trackId;
        } //...for tracks
    } //...problem

    // Make Ceres automatically detect the bundle structure. Note that the
    // standard solver, SPARSE_NORMAL_CHOLESKY, also works fine but it is slower
    // for standard bundle adjustment problems.
    ceres::Solver::Options options;
    options.line_search_direction_type = ceres::NONLINEAR_CONJUGATE_GRADIENT;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.preconditioner_type = ceres::JACOBI;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations           = optParams.maxIterations;
    options.function_tolerance           = 1e-12;
//                  << ", options.gradient_tolerance: " << options.gradient_tolerance
//                  << ", options.parameter_tolerance: " << options.parameter_tolerance
    options.parameter_tolerance           = 1e-16;

    ceres::Solver::Summary summary;
    //ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId ) {
//            int linFrameId = frameId - startFrame;
        Matrix3d R( Matrix3d::Identity() );
        const double* camera = indexer.getCamera(cameras.data(),frameId);
        ceres::AngleAxisToRotationMatrix( camera,
                                          ceres::MatrixAdapter<CeresScalar,
                                              Matrix3d::OuterStrideAtCompileTime,
                                              Matrix3d::InnerStrideAtCompileTime>(R.data()) );
        //R.row(2) *= -1.;
        TransformationT transform = (Eigen::Translation<CeresScalar, 3>(
            Eigen::Map<const Eigen::Matrix<CeresScalar, 3, 1> >(camera + 3)) * R).cast<_Scalar>();
        std::cout << "transform[" << frameId << "]:\n" << transform.matrix() << ",\nR[" << frameId << "]: " << R
                  << ",\nRR'[" << frameId << "]: " << R.inverse() * R << std::endl;

        // object position output
        outTransforms[frameId] = transform;

        if ( frameId == startFrame)
        {
            TrackId trackId( 0 );
            for ( const Track2D& track : tracks2d )
            {
                if ( tracks3d.size() <= trackId )
                {
                    tracks3d.addTrack( Track3D(track.getLabel()) );
                }

                //if ( track.hasPoint(frameId) )
                {
                    const CeresScalar* point = indexer.getPoint( points.data(), trackId );
                    Eigen::Map<const Eigen::Matrix<CeresScalar,3,1> > pnt(point);
                    tracks3d.at( trackId ).addPoint( frameId,
                                                     TrackPoint3D::WithNoNormal( transform * pnt.template cast<Scalar>()) ); // transform is already inverted
                } //...hasPoint()

                ++trackId;
            } //...tracks
        }
    }

    for ( auto const& info : costFunctors )
    {
        const ceres::CostFunction *fn = info.functor;
        CeresScalar* residuals = new CeresScalar[ fn->num_residuals() ];
        fn->Evaluate( info.dataPointers.data(), residuals, NULL );
        std::cout << info.name;
        for ( int ri = 0; ri != fn->num_residuals(); ++ri )
            std::cout << " " << residuals[ri];
        std::cout << std::endl;
        if ( residuals )
        {
            delete [] residuals;
            residuals = nullptr;
        }
    }

    // vis
    {
        Soup::vis::Visualizer<Scalar> vis, vis2;
        vis.addCoordinateSystem( 0.1 );
//            char lineName[255];
        CloudsT clouds( tracks2d.size() );
        for ( int i = 0; i != int(clouds.size()); ++i )
        {
            clouds[i].reset( new CloudT( CloudT(endFrame-startFrame+1)) );
        }

        Vector3 prevTranslation;
        for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId )
        {
            int linFrameId = frameId - startFrame;
            TrackId trackId( 0 );
            for ( const Track2D& track : tracks2d )
            {
                auto const& cloud = clouds.at( trackId );
                if ( track.hasPoint(frameId) )
                    cloud->getPoint(linFrameId) = outTransforms[frameId] * tracks3d.at(trackId).getPoint(startFrame).getPoint();
                if ( track.hasPoint(frameId) )
                    std::cout << "point[" << frameId << "][" << trackId << "] = " << cloud->getPoint(linFrameId) << std::endl;

                // show lines
//                    if ( linFrameId ) // from second onwards
//                    {
//                        sprintf( lineName, "line_frame%u_track_%u", static_cast<unsigned>(frameId), trackId );
//                        vis.addLine( cloud->getPoint(linFrameId-1), cloud->getPoint(linFrameId), colors.at( trackId % colors.size() ) / 255., lineName );
//                        //std::cout << "added " << cloud->getPoint(linFrameId-1) << " -> " << cloud->getPoint(linFrameId) << " as " << lineName << std::endl;
//                    }

                ++trackId;
            } //...for tracks

            Eigen::Matrix3f cam( Eigen::Matrix3f::Identity() * 10. );
            auto const T  = outTransforms.at( frameId ).inverse();
            cam = T * cam;
            for ( int d = 0; d != 3; ++d )
            {
                vis.addArrow( T.translation(), cam.col(d), Vector3(d==0, d==1, d==2), "camOrient" + std::to_string(d) + "_" + std::to_string(frameId), 10. );
            }
            vis.addArrow( Vector3(0.,0.,0.), T.translation(), Vector3(0.,0.,0.), "camera"+std::to_string(frameId), 1. );
            if ( frameId > startFrame )
                vis.addLine( prevTranslation, T.translation(), Vector3(0.,0.,0.), "cameraTrack"+std::to_string(frameId) );
            prevTranslation = T.translation();

        } //...for frames

        int cloudId(0);
        char cloudName[255];
        for ( auto const& cloud : clouds )
        {
            sprintf( cloudName, "cloud_%d", cloudId );
            vis.addPointCloudMono( cloud->getPoints(), colors.at( cloudId % colors.size()), cloudName, nullptr );
            ++cloudId;
        }

        std::cout << "showing" << std::endl;
        std::cout << "intrinsics: "
                  << intrinsics[0] << "," << intrinsics[1]
                  << ", c_xy: " << intrinsics[2] << "," << intrinsics[3]
                  << ", distorts: " << intrinsics[4] << "," << intrinsics[5]
                  << std::endl;
        vis.spin();

        cv::Mat rgb( rgbs.at(4).clone() );
        for ( TrackId trackId = 0; trackId != tracks2d.size(); ++trackId )
        {
            CeresScalar* point = indexer.getPoint( points.data(), trackId );
            cv::Point2i prevPredicted;
            for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId )
            {
                CeresScalar* camera = indexer.getCamera( cameras.data(), frameId );
                CeresScalar p[3];
                ceres::AngleAxisRotatePoint(camera, point, p);

                // camera[3,4,5] are the translation.
                p[0] += camera[3];
                p[1] += camera[4];
                p[2] += camera[5];

#if 0
//                    CeresScalar xp = p[0] / -p[2];
//                    CeresScalar yp = p[1] / -p[2];

//                    const CeresScalar &focalx = intrinsics[0],
//                                      &focaly = intrinsics[1],
//                                      &cx     = intrinsics[2],
//                                      &cy     = intrinsics[3],
//                                      &l1     = intrinsics[4],
//                                      &l2     = intrinsics[5];
//                    const CeresScalar r2 = xp*xp + yp*yp;
//                    const CeresScalar distortion = 1.0 + r2  * (l1 + l2  * r2);


                CeresScalar predicted_x = focalx * xp;// + cx;
                CeresScalar predicted_y = focaly * yp;// + cy;
#else
                CeresScalar predicted_x, predicted_y;
                SnavelyReprojectionError::to2d( predicted_x, predicted_y, p, mapper.getIntrinsics()(0,0), mapper.getIntrinsics()(1,1) );
#endif
                std::cout << "pred: " << predicted_x << "," << predicted_y << std::endl;
                auto color = colors.at( trackId % colors.size() );
                float radius = 3.;
                if ( frameId - startFrame == 4 )
                {
                    color += Eigen::Vector3f( 50., 50., 50. );
                    radius *= 1.5;
                }
                cv::circle( rgb, cv::Point2i(predicted_x,predicted_y), radius, (frameId - startFrame == 4) ? cv::Scalar(0.,0.,0.) : cv::Scalar(color(0),color(1),color(2)) );
                if ( frameId > startFrame )
                {
                    cv::line( rgb, prevPredicted, cv::Point2i(predicted_x,predicted_y), cv::Scalar(color(0),color(1),color(2)) );
                }
                prevPredicted = cv::Point2i(predicted_x,predicted_y);
            } //...for frames
        } //...for tracks
        cv::imshow( "bundler", rgb );
        cv::waitKey();
    } //...vis

    //if ( loss ) { delete loss; loss = nullptr; }

    return 0;
} //...BundleAdjuster::main()
} //...tracking()


