#include "tracking/annot/fitCuboid.h"

#include "opencv2/highgui/highgui.hpp"
#include "soup/visualization/visualizer.h"
#include "soup/visualization/impl/visualizer.hpp"
#include "tracking/common/util/colors.h"
#include "tracking/common/correspondence.h"
#include "Eigen/Dense"
#include "soup/util/util.h" //string_cast
#include <tuple>
//#include "physacq/withAngVel/rigidBody2.hpp"

#define GET_EDGE(edgeId,frameId) ((tracks3d.getTrack( cornersToTracks.at(Cuboid::getEdges()[edgeId+1]) ).getPoint(frameId)) - \
                                  (tracks3d.getTrack( cornersToTracks.at(Cuboid::getEdges()[edgeId  ]) ).getPoint(frameId)))
#define SQR(a) ((a)*(a))

namespace tracking
{
    template <typename _CloudConstPtrT>
    int PCA( Eigen::Matrix4f &frame,
             _CloudConstPtrT cloud
           )

    {
        //static_assert( _CloudConstPtrT::element_type::ColsAtCompileTime == 3, "PCA Assumed points in rows" );
        typedef float Scalar;
        Eigen::Matrix<Scalar,1,3> centroid = cloud->getCentroid();
        auto tmp = cloud->getPoints().rowwise() - centroid;
        std::cout << tmp.transpose() * tmp << std::endl;
        Eigen::Matrix3f covariance = (tmp.transpose() * tmp).eval();
        //pcl::computeCovarianceMatrixNormalized( *cloud, *indices_ptr, centroid, covariance );

        // eigen
        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver( covariance, Eigen::ComputeEigenvectors );
        Eigen::Matrix3f eigen_vectors = eigen_solver.eigenvectors();
        std::set< std::pair<float, int> > sorted = { {eigen_solver.eigenvalues()(0),0}, {eigen_solver.eigenvalues()(1),1}, {eigen_solver.eigenvalues()(2),2} };
        std::vector< int > indices;
        for ( auto it = sorted.rbegin(); it != sorted.rend(); ++it )
            indices.emplace_back( it->second );

        eigen_vectors.col( indices[2] ) = eigen_vectors.col(indices[0]).cross( eigen_vectors.col(indices[1]) );

        // debug
        if ( eigen_vectors(0,0) != eigen_vectors(0,0)
             || eigen_vectors(1,1) != eigen_vectors(1,1)
             || eigen_vectors(2,2) != eigen_vectors(2,2) )
        {
            std::cerr << "nan eigen matrix" << std::endl;
            return EXIT_FAILURE;
        }

        frame = Eigen::Matrix4f::Identity();
        for ( int d = 0; d != 3; ++ d )
            frame.col( d ).head<3>() = eigen_vectors.col( indices.at(d) );
        frame.block<3,1>(0,3) = centroid.head<3>();
    } //PCA



    inline typename Soup::ColoredPointCloud< typename Tracks3D::Scalar >::PtrT frame2Cloud( const Tracks3D tracks3d, const FrameId frameId )
    {
        typedef typename Soup::ColoredPointCloud< typename Tracks3D::Scalar > CloudT;
        typedef typename CloudT::PtrT CloudPtrT;

        std::vector< TrackPoint3D > points;
        for ( auto const& track : tracks3d )
        {
            if ( track.hasPoint(frameId) )
            {
                points.emplace_back( track.getPoint(frameId) );
            }
        }
        CloudPtrT cloud( new CloudT(points.size()) );
        for ( size_t pId = 0; pId != points.size(); ++pId )
        {
            cloud->getPoint ( pId ) = points[ pId ].getPoint();
            cloud->getNormal( pId ) = points[ pId ].getNormal();
        }
        return cloud;
    } //...track2PCloud()

    int Fitter::fitCuboid(       Tracks3D    &tracks3d,
                                 TransformsT &outTransforms,
                           const TransformsT &inTransforms,
                           const Tracks2D    &tracks2d,
                           const Mapper      &mapper,
                           const Weights     &weights,
                           const int          width,
                           const int          height,
                           const FrameId      startFrame,
                           const FrameId      endFrame, // inclusive!
                           const std::vector< ::Eigen::Vector3f >& colors,
                           const std::vector<Cuboid>             * cuboids,
                                 std::vector<Cuboid>             * outCuboids
                         )
    {
        using ceres::FunctorInfo;
        const Tracks2D::CountT trackCount( tracks2d.getTrackCount() );
        typedef typename TransformsT::mapped_type TransformT;
        typedef typename Tracks2D::Scalar Scalar;
        typedef Soup::ColoredPointCloud<Scalar> CloudT;

        std::vector< FunctorInfo >  costFunctors;
        ceres::Problem              problem;
        ceres::LossFunction* loss = new ceres::HuberLoss( 1. );

        std::cout << "[" << __func__ << "]: " << "weights: "
                  << "uv: " << weights.uvWeight
                  << ", rigid:" << weights.rigidWeight
                  << ", parallel:" << weights.paralWeight
                  << ", perpendicular: "<< weights.perpWeight
                  << std::endl;

        size_t span   ( endFrame - startFrame + 1),
               nPoints( span * trackCount );
        std::vector< CeresScalar >  uvzs( nPoints * Indexer::UVZS_STRIDE );
        std::transform( uvzs.begin(), uvzs.end(), uvzs.begin(), []( const CeresScalar& v ) { return rand()/static_cast<float>(RAND_MAX) * 2.; } );

        Indexer          indexer( startFrame, trackCount );
        std::vector<int> cornersToTracks; // track = tracks2d[ trackIds[cornerId] ]

        // propagate
        FrameId          knownFrameId = -1;
        Cuboid::CornersT corners3d; // corners of cuboid in 3D at known timepoint knownFramId
        if ( cuboids )
        {
            auto const& cuboid = cuboids->at(0);
            knownFrameId = startFrame;
            while ( !cuboid.hasFrame(knownFrameId) && (knownFrameId <= endFrame) )
                ++knownFrameId;
            if ( knownFrameId > endFrame )
            {
                std::cerr << "[" << __func__ << "]: " << "no known cuboid positions, can't align transforms!" << std::endl;
                throw new Fitter_NoKnownCuboidPoseGivenException("no pose");
            }

            auto const  T = cuboid.getTransformation(knownFrameId);
            std::vector<Eigen::Vector2f> corners2d;
            getCorners2D( corners2d, T, mapper );
            corners3d = T * *Cuboid::getCorners();
            if ( !cuboid.matchTrackIds( /* out: */ cornersToTracks, /* in: */ corners2d,tracks2d,knownFrameId) )
            {
                throw new Fitter_TrackToCornerAssignmentIsNotUniqueException("not unique");
            }
            std::cout << "cuboid: " << cuboid.getSize().transpose() << ",\n" << corners3d << ",\nT" << T.matrix() << std::endl;
        } //...select known cuboid

        std::set< std::pair<size_t,size_t> > edges;
        {
            for ( int i = 0; i != tracks2d.getTrackCount()-1; ++i )
                for ( int j = i+1; j < tracks2d.getTrackCount(); ++j )
                {
                    //if ( rand()/static_cast<float>(RAND_MAX) < 0.2 )
                        edges.insert( std::make_pair(i,j) );
                }
//            while ( edges.size() < 12 )
//            {
//                size_t first =  rand() % tracks2d.getTrackCount(), second;
//                while ( first == (second = rand() % tracks2d.getTrackCount()) );
//                edges.insert( std::make_pair(first,second) );
//            } //...while not enough edges
        }

        char functorName[255];
        for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId )
        {
            // to observation:
            //double depth = rand() / static_cast<double>(RAND_MAX) * 2.;
            for ( TrackId trackId = 0; trackId != tracks2d.getTrackCount(); ++trackId )
            {
                const Track2D& track = tracks2d.getTrack( trackId );
                if ( track.hasPoint(frameId) )
                {
                    auto const& trackPoint = track.getPoint(frameId);
                    // 2D
                    {
                        ceres::CostFunction* costFunction =
                                new ceres::AutoDiffCostFunction<OnTrackFunctorT, /* num_residuals: */ 3, /* num_params: */ 3>(
                                    new OnTrackFunctorT(mapper.getIntrinsics(), trackPoint(0), trackPoint(1), width, height, weights.uvWeight) );
                        CeresScalar* address = indexer.getUnknownAddress(uvzs, frameId, trackId);
                        problem.AddResidualBlock( costFunction, nullptr, address );
                        if ( indexer.getPId(frameId, trackId) >= nPoints )
                            std::cerr << "[" << __func__ << "]: " << "baaamm: f" << frameId << ", t" << trackId << ", tc" << trackCount << ", n" << nPoints
                                      << ", pid" << indexer.getPId(frameId, trackId) << std::endl;

                        // reporting
                        sprintf( functorName, "OnTrackCostFunctor_track_%u_time%u", trackId, static_cast<unsigned>(frameId) );
                        costFunctors.emplace_back( FunctorInfo(functorName, costFunction, {address}) );

                        // INIT
                        {
                            address[0] = trackPoint(0);
                            address[1] = trackPoint(1);
                            if ( cuboids && cornersToTracks.size() )
                            {
                                auto cornerIt = std::find( cornersToTracks.begin(), cornersToTracks.end(), trackId );
                                if ( cornerIt != cornersToTracks.end() )
                                {
                                    int cornerId = std::distance( cornersToTracks.begin(), cornerIt );
                                    address[2] = corners3d( 2, cornerId );

                                    if ( frameId == knownFrameId )
                                    {
                                        // sanity: init should match cuboid
                                        double xy[2];
                                        getXy( address, xy, mapper.getIntrinsics() );
                                        if ( SQR(xy[0]-corners3d(0,cornerId)) + SQR(xy[1]-corners3d(1,cornerId)) + SQR(address[2]-corners3d(2,cornerId)) > 0.01 )
                                        {
                                            std::cerr << "[" << __func__ << "]: " << "cuboid - init:"
                                                      << "\n\t" << xy[0] << " vs. " << corners3d( 0, cornerId )
                                                      << "\n\t" << xy[1] << " vs. " << corners3d( 1, cornerId )
                                                      << "\n\t" << address[2] << " vs. " << corners3d( 2, cornerId )
                                                      << std::endl;
                                        } //...if init didn't work
                                    } //...if cuboid can be inited
                                } //...if not last corner
                            } //...if cuboids
                            else if ( inTransforms.find(frameId) != inTransforms.end() )
                                address[2] = inTransforms.at(frameId).translation()(2);
                            //address[2] = init.at( frameId - startFrame )(2);
                            //uvzs[(uvzsOffset + t)*UVZS_STRIDE+2] = curve.getPoint(frameId)(2);

                        } //...INIT
                    } //...2D
                } //...hasPoint
            } //...for tracks
#if 1
            // constrain to othertracks
            if ( frameId > startFrame )
            {
                for ( auto const& edge : edges )
                {
                    CeresScalar* prev0 = indexer.getUnknownAddress( uvzs, startFrame, /* trackId: */ edge.first  );
                    CeresScalar* prev1 = indexer.getUnknownAddress( uvzs, startFrame, /* trackId: */ edge.second );
                    CeresScalar* curr0 = indexer.getUnknownAddress( uvzs, frameId  , /* trackId: */ edge.first  );
                    CeresScalar* curr1 = indexer.getUnknownAddress( uvzs, frameId  , /* trackId: */ edge.second );

                    ceres::CostFunction* costFunction =
                            new ceres::AutoDiffCostFunction<EqualDistanceFunctorT, /* num_residuals: */ 4, /* num_params: */ 3, /* num_params: */ 3, /* num_params: */ 3, /* num_params: */ 3 >(
                                new EqualDistanceFunctorT( mapper.getIntrinsics(), weights.rigidWeight, 0.001 ) );
                    problem.AddResidualBlock( costFunction, NULL, prev0, curr0, prev1, curr1  );

                    sprintf( functorName, "EqualDistance_track%lu-%lu_time%u", edge.first, edge.second, static_cast<unsigned>(frameId) );
                    costFunctors.emplace_back( FunctorInfo(functorName, costFunction, {prev0,curr0,prev1,curr1}) );
                }
            }
#endif
#if 1
            // perpendicular
            if ( cuboids )
            {
                // perpendicular
                for ( int pivotId = 0; pivotId < Cuboid::getParallelEdges().at(0).size(); pivotId+=2 )
                {
                    const Cuboid::EdgesT& pivotEdges = Cuboid::getParallelEdges().at( 0 );
                    CeresScalar* p0 = indexer.getUnknownAddress( uvzs, frameId, cornersToTracks[pivotEdges[pivotId  ]] );
                    CeresScalar* p1 = indexer.getUnknownAddress( uvzs, frameId, cornersToTracks[pivotEdges[pivotId+1]] );
                    for ( int paralId = 1; paralId != Cuboid::getParallelEdges().size(); ++paralId )
                    {
                        const Cuboid::EdgesT& parallelEdges = Cuboid::getParallelEdges().at( paralId );
                        for ( int edgeId = 0; edgeId < parallelEdges.size(); edgeId += 2 )
                        {
                            CeresScalar* q0 = indexer.getUnknownAddress( uvzs, frameId, cornersToTracks[parallelEdges[edgeId  ]] );
                            CeresScalar* q1 = indexer.getUnknownAddress( uvzs, frameId, cornersToTracks[parallelEdges[edgeId+1]] );
                            ceres::CostFunction* costFunction = nullptr;
                            if ( p0 == q0 || p0 == q1 || q0 == p1 || p1 == q1 )
                            {
                                int aid, bid, cid;
                                CeresScalar *a,*b,*c; // a -- b, a -- c
                                if      ( p0 == q0 ) { a = p0; b = p1; c = q1; aid = pivotEdges   [pivotId  ]; bid = pivotEdges[pivotId+1]; cid = parallelEdges[edgeId+1]; }
                                else if ( p0 == q1 ) { a = p0; b = p1; c = q0; aid = pivotEdges   [pivotId  ]; bid = pivotEdges[pivotId+1]; cid = parallelEdges[edgeId  ]; }
                                else if ( p1 == q1 ) { a = p1; b = p0; c = q0; aid = pivotEdges   [pivotId+1]; bid = pivotEdges[pivotId  ]; cid = parallelEdges[edgeId  ]; }
                                else if ( p1 == q0 ) { a = q0; b = p0; c = q1; aid = parallelEdges[edgeId   ]; bid = pivotEdges[pivotId  ]; cid = parallelEdges[edgeId+1]; }

                                costFunction =
                                        new ceres::AutoDiffCostFunction<DotProductFrom3PointsFunctorT, /* num_residuals: */ 1, /* num_params: */ 3, /* num_params: */ 3, /* num_params: */ 3 >(
                                            new DotProductFrom3PointsFunctorT( mapper.getIntrinsics(), /* targetDot: */ 0., weights.perpWeight ) );
                                problem.AddResidualBlock( costFunction, NULL, a,b,c );
                                sprintf( functorName, "Perpendicular[3]_%d-%d_%d-%d_time%u", aid, bid, aid, cid, static_cast<unsigned>(frameId) );
                                costFunctors.emplace_back( FunctorInfo(functorName, costFunction, {a,b,c}) );
                            }
                            else
                            {
                                costFunction =
                                    new ceres::AutoDiffCostFunction<DotProductFunctorT, /* num_residuals: */ 1, /* num_params: */ 3, /* num_params: */ 3, /* num_params: */ 3, /* num_params: */ 3 >(
                                        new DotProductFunctorT( mapper.getIntrinsics(), /* targetDot: */ 0., weights.perpWeight ) );
                                problem.AddResidualBlock( costFunction, NULL, p0,p1, q0,q1  );
                                sprintf( functorName, "Perpendicular[4]_%d-%d_%d-%d_time%u", pivotEdges[pivotId],pivotEdges[pivotId+1],parallelEdges[edgeId],parallelEdges[edgeId+1], static_cast<unsigned>(frameId) );
                                costFunctors.emplace_back( FunctorInfo(functorName, costFunction, {p0,p1,q0,q1}) );
                            }
                        } //...edges
                    } //...paral groups
                } //...pivot edges
            } //...if cuboids
#endif
#if 1
            if ( cuboids && cuboids->at(0).hasFrame(frameId) )
            {
                {
                    auto edges = Cuboid::getEdges();
                    for ( int j = 0; j < edges.size(); j += 2)
                    {
                        const int cornerId0 = edges.at(j);
                        const int cornerId1 = edges.at(j+1);

                        const TrackId trackId0 = cornersToTracks.at( cornerId0 );
                        const TrackId trackId1 = cornersToTracks.at( cornerId1 );
                        CeresScalar* curr0 = indexer.getUnknownAddress( uvzs, frameId, /* trackId: */ trackId0 );
                        CeresScalar* curr1 = indexer.getUnknownAddress( uvzs, frameId, /* trackId: */ trackId1 );
                        CeresScalar distance = (corners3d.col( cornerId0 ) - corners3d.col( cornerId1 )).norm();
                        ceres::CostFunction* costFunction =
                                new ceres::AutoDiffCostFunction<FixedDistanceFunctorT, /* num_residuals: */ 1, /* num_params: */ 3, /* num_params: */ 3 >(
                                    new FixedDistanceFunctorT( mapper.getIntrinsics(), distance, weights.fixedWeight ) );
                        problem.AddResidualBlock( costFunction, NULL, curr0, curr1  );

                        sprintf( functorName, "FixedDistance_track%d-%d_time%u", edges.at(j), edges.at(j+1), static_cast<unsigned>(frameId) );
                        costFunctors.emplace_back( FunctorInfo(functorName, costFunction, {curr0,curr1}) );
                    }
                }
            } //...if has cuboids
#endif
        } //...for all frames

        /// Run the solver
        ceres::Solver::Options options;
        {
            options.linear_solver_type           = ceres::SPARSE_NORMAL_CHOLESKY;
            options.preconditioner_type          = ceres::JACOBI;
            options.minimizer_progress_to_stdout = true;
            options.max_num_iterations           = 2000;

            // defaults:
            //  options.function_tolerance: 1e-06
            //  options.gradient_tolerance: 1e-10
            // options.parameter_tolerance: 1e-08
            std::cout << "options.function_tolerance: " << options.function_tolerance
                      << ", options.gradient_tolerance: " << options.gradient_tolerance
                      << ", options.parameter_tolerance: " << options.parameter_tolerance
                      << std::endl;
        }

        ceres::Solver::Summary summary;
        ceres::Solve(options, &problem, &summary);
        std::cout << summary.BriefReport() << "\n";


        typedef Tracks2D::Scalar Scalar;
        Vector3 sidesDecreasing(0.,0.,0.);
        if ( 1 )
        {
            Soup::vis::Visualizer<Scalar> vis( "optimizeTrack" );
            vis.setBackgroundColor( 1., 1., 1. );
            vis.addCoordinateSystem( 0.05 );

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

            // show tracks
            {
                std::map< FrameId, CorrPairs > corrs;
                //tracking::CorrPairsList corrs; // one list for each pair of consequtive frames
                //Tracks3D tracks3d;
                char cloudName[255];
                for ( TrackId trackId = 0; trackId != tracks2d.getTrackCount(); ++trackId )
                {
                    Track3D track3d;

                    //std::vector<Eigen::Vector3d> points;
                    for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId )
                    {
                        CeresScalar* uvz = indexer.getUnknownAddress( uvzs, frameId, /* trackId: */ trackId );
                        Eigen::Vector3d pnt;
                        //points.push_back( Eigen::Vector3d() );
                        getXy( uvz, pnt.data(), mapper.getIntrinsics() );
                        pnt(2) = uvz[2];
                        track3d.addPoint( frameId, TrackPoint3D::WithNoNormal(pnt.cast<Scalar>()) );

                        // connect trackpoints in same track
                        //if ( frameId > startFrame )
                        {
                            // relTransforms[frameId] will hold the transform how you arrive from knownFrameId -> frameId.
                            corrs[ frameId ].emplace_back( CorrPair( TrackIdFrameId(trackId,startFrame),
                                                                     TrackIdFrameId(trackId,frameId   )) );
                        }
                    } //...frames
                    tracks3d.addTrack(track3d);

//                    Soup::PointCloud<Scalar> cloud( points.size() );
//                    size_t row(0);
//                    for ( auto const& point : points )
//                    {
//                        cloud.getPoint(row) = point.cast<Scalar>();
//                        ++row;
//                    }
                    auto cloud = track3d.getCloud<CloudT>();
                    sprintf( cloudName, "cloud%u", trackId );

                    vis.addPointCloudMono( cloud->getPoints(), colors.at( trackId % colors.size() ), cloudName );

                } //...tracks

                // estimate transforms
                {
                    TransformsT relTransforms;
                    int linId = 0;
                    for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId, ++linId )
                    {
                        auto corr = corrs.at(frameId);
                        Correspondence::TransformationT transform;
                        Correspondence::computeTransform( transform, tracks3d, corr );
                        // relTransforms[frameId] holds the transform how you arrive from frameId-1 -> frameId.
                        relTransforms[ frameId ] = transform;

//                        char lineName[255];
//                        for ( auto const pair : corr )
//                        {
//                            sprintf( lineName, "line_%u_%u_%u", pair.first.getTrackId(), pair.first.getFrameId(), pair.second.getFrameId() );
//                            vis.addLine( tracks3d.getTrack(pair.first.getTrackId()).getPoint(pair.first.getFrameId()),
//                                         tracks3d.getTrack(pair.second.getTrackId()).getPoint(pair.second.getFrameId()),
//                                         colors.at( pair.first.getTrackId() % colors.size() ) / 255.,
//                                         lineName );
//                        }
                    } //...for frames

                    // show cuboids
                    if ( cuboids && cornersToTracks.size() )
                    {
                        for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId, ++linId )
                        {
                            char lineName[255];
                            for ( int edgeId = 0; edgeId < Cuboid::getEdges().size(); edgeId+=2 )
                            {
                                auto const cornerId0 = cornersToTracks.at( Cuboid::getEdges()[edgeId  ] );
                                auto const cornerId1 = cornersToTracks.at( Cuboid::getEdges()[edgeId+1] );
                                //std::cout << "edge" << edgeId << ": " << Cuboid::getEdges()[edgeId  ] << "," << Cuboid::getEdges()[edgeId+1] << " = " << cornerId0 << "," << cornerId1 << std::endl;
                                auto const p0 = tracks3d.getTrack( cornerId0 ).getPoint( frameId );
                                auto const p1 = tracks3d.getTrack( cornerId1 ).getPoint( frameId );
                                sprintf( lineName, "edge_%d_%d_%u", cornerId0, cornerId1, static_cast<unsigned>(frameId) );
                                vis.addLine( p0.getPoint(), p1.getPoint(), Eigen::Vector3f(0.,0.,0.), lineName );
                                //std::cout << lineName << p0.transpose() << " - " << p1.transpose() << std::endl;
                            } //...for edges
                        } //...for each frame
                    } //...if has cuboid

                    // propagate
                    if ( cuboids )
                    {
//                        auto const& cuboid = cuboids->at(0);
//                        TransformT knownTransform = Eigen::Translation<Scalar,3>( cuboid.getPosition(knownFrameId)                    ) *
//                                                    TransformT                  ( cuboid.getPose    (knownFrameId).toRotationMatrix() );

                        auto firstCloud = frame2Cloud( tracks3d, startFrame );
                        Eigen::Matrix4f firstT;
                        PCA( firstT, firstCloud );
                        outTransforms[ startFrame ] = firstT;
                        // relTransforms[frameId] holds the transform how you arrive from frameId-1 -> frameId.
                        for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId )
                        {
                            outTransforms[frameId] = relTransforms.at( frameId ) * firstT;
                        }

                        // get scale
                        {
                            std::set<float> edgeLengths;

                            for ( int edgeId = 0; edgeId != static_cast<int>(Cuboid::getEdges().size()); edgeId+=2 )
                            {
                                edgeLengths.insert( GET_EDGE( edgeId, knownFrameId ).norm() );
                            }
                            std::cout << "edgeLengths: ";
                            for ( auto const len : edgeLengths )
                                std::cout << len << ",";
                            std::cout << std::endl;

                            auto it = edgeLengths.rbegin();
                            for ( int i = 0; it != edgeLengths.rend(); ++i )
                            {
                                for ( int j = 0; j != 4; ++j, ++it )
                                    sidesDecreasing(i) += *it;
                                sidesDecreasing(i) /= 4.;
                            }
                        }

                        Eigen::Matrix4f S( Eigen::Matrix4f::Identity() );
                        S.block<3,3>(0,0) = sidesDecreasing.asDiagonal();

#if 0
                        char edgeName[255];
                        for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId )
                        {
                            auto TS = outTransforms[frameId] * S;
                            Cuboid::CornersT corners = *Cuboid::getCorners();
                            for ( int col = 0; col != Cuboid::getCorners()->cols(); ++col )
                            {
                                corners.col(col) = TS * Cuboid::getCorners()->col(col);
                            }
                            for ( int edgeId = 0; edgeId != Cuboid::getEdges().size(); edgeId += 2 )
                            {
                                sprintf( edgeName, "cuboid[%u][%d-%d]", frameId, edgeId, edgeId+1 );
                                vis.addLine( corners.col(Cuboid::getEdges().at(edgeId)).template head<3>(), corners.col(Cuboid::getEdges().at(edgeId+1)).template head<3>(), Vector3(0.,0.,1.), edgeName );
                            }
                            //vis.spin();
                        } // blue rectangle for each frame
#endif
                    } //...propagate

                    // show trajectory
                    Vector3 prevCentroid( Vector3::Zero() );
                    for ( FrameId frameId = startFrame; frameId <= endFrame; ++frameId )
                    {
                        Vector3 centroid( Vector3::Zero() );
                        for ( TrackId trackId = 0; trackId != tracks3d.size(); ++trackId )
                            centroid += tracks3d.getTrack( trackId ).getPoint( frameId ).getPoint();
                        centroid /= static_cast<float>( tracks3d.size() );
                        if ( frameId > startFrame )
                        {
                            char lineName[255];
                            sprintf( lineName, "centroid_%u", static_cast<unsigned>(frameId) );
                            vis.addArrow( prevCentroid,
                                         centroid,
                                         Vector3(1.,0.,0.),
                                         lineName, 0.01 );
                        }
                        prevCentroid = centroid;
                    }
                } //...
            } //show

            if ( cuboids )
            {
                for ( int cId = 1; cId < cuboids->size(); ++cId )
                {
                    auto const& cuboid = cuboids->at(cId);
                    //for ( auto const& )
                    TransformT T = Eigen::Translation<Scalar,3>( cuboid.getPosition(knownFrameId)                    ) *
                                   TransformT                  ( cuboid.getPose    (knownFrameId).toRotationMatrix() );
                    TransformT S( TransformT::Identity() );
                    S.matrix().block<3,3>(0,0) = cuboid.getSize().asDiagonal();
                    drawCuboid( vis, T * S, Vector3(0.,0.,0.), "cuboid" + std::to_string(cId) );
                }
            }
            vis.spin();

            // /home/bontius/workspace/physAcq/aligner/include/aligner/quaternion.hpp
            std::map< FrameId, PoseLoc > velOmegas;
            auto const* prevTransform = &outTransforms.at( startFrame );
            for ( FrameId frameId = startFrame+1; frameId <= endFrame; ++frameId )
            {
                auto const& transform = outTransforms.at( frameId ); // TODO: reltransform!!
                if ( frameId != startFrame )
                {
                    velOmegas[ frameId ] = PoseLoc( transform.translation() - prevTransform->translation(),
                                                    Cuboid::QuaternionT( 0., pa::getOmega( Cuboid::QuaternionT(transform     .rotation()),
                                                                                           Cuboid::QuaternionT(prevTransform->rotation()) ) ) );
                }
                prevTransform = &transform;
            }

            // midpoint smooth velocities
            {
                // velocity_{i-1 -> i  } is stored at velOmega[i  ]->getPosition()
                // velocity_{i   -> i+1} is stored at velOmega[i+1]->getPosition()
                // new velocity_{i} = (velocity_{i-1 -> i} + velocity_{i -> i+1} ) /2.

                auto it0 = velOmegas.begin(), it1 = velOmegas.begin();
                ++it1;
                Vector3 tmp = it0->second.getPosition();
                for ( ; it1 != velOmegas.end(); ++it0, ++it1 )
                {
                    Vector3 newVel = (tmp + it1->second.getPosition()) / Scalar(2.);
                    tmp = it0->second.getPosition(); // save velocity_{i-1 -> i}
                    it0->second.getPosition() = newVel;
                }
            }

            //applyOmega
            if ( outCuboids )
            {
                Soup::vis::Visualizer<Scalar> vis;
                vis.setBackgroundColor( 1., 1., 1. );

                Cuboid cuboid( Vector3(sidesDecreasing[0],sidesDecreasing[1],sidesDecreasing[2] ) );
                std::cout<<"out_lengths:";for(size_t vi=0;vi!=sidesDecreasing.size();++vi)std::cout<<sidesDecreasing[vi]<<" ";std::cout << "\n";
                Vector3 loc0 = frame2Cloud( tracks3d, startFrame )->getCentroid();
                Cuboid::QuaternionT pose( outTransforms.at(startFrame).rotation() );
                cuboid.addState( startFrame, PoseLoc(outTransforms.at(startFrame).translation(), pose) );
                for ( auto const& pair : velOmegas )
                {
                    const FrameId  frameId  = pair.first;
                    const PoseLoc& velOmega = pair.second;
                    const PoseLoc& prevState = cuboid.getState( frameId - 1 );
                    pa::applyOmega( pose, velOmega.getPose(), Scalar(1.), 10 );
                    cuboid.addState( frameId, PoseLoc(prevState.getPosition() + velOmega.getPosition(), pose ) );
                    cuboid.setAngVel( frameId, velOmega.getPose().coeffs().template head<3>() );
                    cuboid.setLinVel( frameId, velOmega.getPosition() );
                    //cuboid.addState( frameId, PoseLoc(prevState.getPosition() + velOmega.getPosition(),
                    //                                  Cuboid::QuaternionT(outTransforms.at(frameId).rotation()) ) );
                }
                outCuboids->push_back( cuboid );

                vis.spin();
            }
        } // ...if doVis
    } //...fitCuboid()
} //...ns annot

#undef GET_EDGE
#undef SQR




