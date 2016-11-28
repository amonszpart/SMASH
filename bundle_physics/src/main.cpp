#include "tracking/common/eigen.h"

#include "tracking/phys/trackingLogger.h"
#include "tracking/phys/bundleWithPhysics.h"
#include "tracking/phys/infMass.h"
#include "tracking/phys/tripleColl.h"
#include "tracking/phys/preprocessCuboids.h"
#include "tracking/phys/weights.h"
#include "tracking/phys/vis/visPhys.h"
#include "tracking/phys/io/parabolaIo.h"
#include "tracking/phys/io/weightsIo.h"

#include "tracking/annot/annotator.h"
#include "tracking/annot/cuboid.h"
#include "tracking/annot/cuboidFwDecl.h"

#include "tracking/common/io/sequentialReader.h"
//#include "tracking/common/io/broxIO.h"
#include "tracking/common/io/rgbIo.h"              // read_images
#include "tracking/common/io/optionsParser.h"
#include "tracking/common/io/segmentedTracksIo.h"
#include "tracking/common/io/os.h"
#include "tracking/common/io/ceresOptionsIo.h"
#include "tracking/common/optParams.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/mapper.h"
#include "tracking/common/util/colors.h"
#include "tracking/common/util/impl/parse.hpp"     // console
#include "tracking/common/typedefs.h"

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "sys/stat.h" // mkdir
#include <iostream>
#include <string>
#include <bitset>
#include <map>
#include <vector>
#include <stdlib.h>

namespace tracking {
  namespace bundle_physics {

//    int testQuat(int argc, char **argv);

    DEFINE_EXCEPTION(Merge_DuplicateEntry)

    int merge(int argc, const char **argv) {

        auto printUsage = []() { std::cerr << "usage: --merge --cub c0.json,c1.json --out out.json"; };
        std::vector<std::string> paths;
        console::parse_x_arguments(argc, argv, "--cub", paths);
        if (!paths.size()) {
            printUsage();
            return EXIT_FAILURE;
        }
        std::string outPath;
        if (!console::parse_arg(argc, argv, "--out", outPath)) {
            std::cerr << "--out missing" << std::endl;
            printUsage();
            return EXIT_FAILURE;
        }

        CuboidsT in, accum;
        for (std::string const &path : paths) {
            in.clear();
            io::readCuboids(in, path);
            for (const Cuboid &inCuboid : in) {
                bool        merged = false;
                for (Cuboid &cuboid : accum) {
                    if (cuboid.getName().compare(inCuboid.getName()) == 0) {
                        for (auto const &pair : inCuboid.getStates()) {
                            if (cuboid.hasFrame(pair.first))
                                throw new Merge_DuplicateEntryException("no");
                            else
                                cuboid.addState(pair.first, pair.second);
                        }
                        merged = true;
                        break;
                    }
                }
                if (!merged)
                    accum.push_back(inCuboid);
            }
        }

        io::writeCuboids(accum, outPath);
        return EXIT_SUCCESS;
    } //...merge

    extern int anim2(CuboidsT const& cuboids, FrameIdsT const& frameIds, Mapper const& mapper, Weights weights, LinRgbsT const& rgbs, int argc, const char** argv);

  } //...ns bundle_physics
} //...ns tracking

// cx: 60.000000, cy: 60.000000, fx: 131.250000, fy: 131.250000
int convertPng2Ppm(int argc, const char** argv) {
    int index = console::find_argument(argc,argv,"--png2ppm" );
    if (index < 1 || index+3 >= argc || console::find_switch(argc,argv,"-h")) {
        std::cerr << "usage: --png2ppm in.png out.ppm res_x [res_y]" << std::endl;
        return EXIT_FAILURE;
    }
    bool doScale(false);
    std::string inPath;
    console::parse_var(argv[index+1], inPath);
    std::cout << "[" << __func__ << "] " << "parsed argv[" << index+1 << "]: " << argv[index+1] << std::endl;
    std::string outPath;
    console::parse_var(argv[index+2], outPath);
    std::cout << "[" << __func__ << "] " << "parsed argv[" << index+2 << "]: " << argv[index+2] << std::endl;

    cv::Mat img = cv::imread(inPath, cv::IMREAD_UNCHANGED);

    int res_x = img.cols;
    doScale |= console::parse_var(argv[index+3], res_x);
    int res_y = img.rows;
    if (index+4 < argc) {
        doScale |= console::parse_var(argv[index + 4], res_y);
    } else
        res_y = std::round(static_cast<float>(img.rows) * static_cast<float>(res_x) / static_cast<float>(img.cols));

    if (doScale) {
        cv::Mat scaled(res_y, res_x, img.type());
        std::cout << "[" << __func__ << "] " << "scaling to " << scaled.size() << std::endl;
        cv::resize(img, scaled, scaled.size(), 0., 0., cv::INTER_LANCZOS4);
        tracking::writeRgb(scaled, outPath);
    } else {
        tracking::writeRgb(img, outPath);
    }
    std::cout << "[" << __func__ << "]: " << "wrote to " << outPath << std::endl;
    return EXIT_SUCCESS;
}

extern int quatTest(int argc, const char **argv);

int main(int argc, const char *argv[]) {
    using namespace tracking::bundle_physics;
    using tracking::bundle_physics::GroupsCuboidsT;
    using tracking::bundle_physics::CuboidsT;
    using tracking::bundle_physics::Vector3;
    using tracking::bundle_physics::Cuboid;
    using ceres::CeresScalar;
    using ceres::CeresVector3;
//    return quatTest(argc,argv);

    google::InitGoogleLogging(argv[0]);
    int ret = EXIT_SUCCESS;

    // --png2ppm
    if (console::find_switch(argc,argv,"--png2ppm"))
        return convertPng2Ppm(argc,argv);

    // --testQuat
    //return tracking::testQuat( argc, argv );

    // --merge
    if (console::find_switch(argc,argv,"--merge"))
        return tracking::bundle_physics::merge(argc,argv);

    // --mov
    {
        std::string movPath;
        if (console::parse_arg(argc, argv, "--mov", movPath))
        {
            cv::VideoCapture vc(movPath);

            cv::Mat tmp;
            int     frameId(0);
            char    outPath[255];
            while (vc.read(tmp))
            {
                //cv::imshow( "mov", tmp );
                sprintf(outPath, "color_%05d.ppm", frameId);
                tracking::writeRgb(tmp, outPath);
                //cv::waitKey(10);
                ++frameId;
            }
            return 0;
        }
    }

    // read frameIds
    tracking::FrameIdsT frameIds;
    {
        std::vector<unsigned> tmpFrameIds;
        if (console::parse_x_arguments(argc,argv,"--frames",tmpFrameIds) < 0) {
            std::cerr << "[" << __func__ << "]: "
                      << "you should specify the input frame ids by --frames start,collTimeLower,end" << std::endl;
            return 1;
        } else
            frameIds.insert( frameIds.end(), tmpFrameIds.begin(), tmpFrameIds.end() );
    }

    // intrinsics
    tracking::Mapper mapper(tracking::Mapper::DSLR_NILOY);
    if (EXIT_SUCCESS != tracking::parseMapper(mapper, argc, argv)) {
        std::cerr << "need to specify intrinsics with --intr" << std::endl;
        return EXIT_FAILURE;
    }

    // images
    tracking::LinRgbsT rgbs;
    if (tracking::readImages(rgbs,argc,argv, frameIds)) {
        std::cerr << "[" << __func__ << "]: " << "readImages returned an error..." << std::endl;
        return EXIT_FAILURE;
    }

    // --max-iterations
    tracking::OptParams optParams;
    console::parse_arg(argc, argv, "--max-iterations", optParams.maxIterations);

    // cuboids
    std::vector<std::string> cuboidsPaths;
    CuboidsT cuboids;
    if (console::parse_x_arguments(argc,argv,"--cub",cuboidsPaths)) {
        CuboidsT cuboidsAll, cuboidsInfinite;
        tracking::bundle_physics::io::readCuboids(cuboidsAll, cuboidsPaths);
        for (auto const& cuboid : cuboidsAll) {
            if (cuboid.isMassFinite())
                cuboids.push_back(cuboid);
            else
                cuboidsInfinite.push_back(cuboid);
        }
        for (auto const& cuboid : cuboidsInfinite)
            cuboids.push_back(cuboid);
//#warning "UNCOMMENTED ORIENTCUBOIDS"
        orientCuboids(cuboids);

        std::cout << "[" << __func__ << "] " << "have " << cuboids.size() << "cuboids: ";
        for (auto const& cuboid : cuboids) {
            std::cout << "states: " << cuboid.getStates().size() << ",";
        }
        std::cout << std::endl;
    }

    // parameters
    tracking::bundle_physics::Weights weights;
    {
        std::tie(weights, ret) = tracking::bundle_physics::io::readWeights(argc, argv);
        if (EXIT_SUCCESS != ret) {
            std::cerr << "[" << __func__ << "] reading weights failed" << std::endl;
            return ret;
        }
        std::vector<int> solveStages; // 0: momenta, 1: coupled, 2: free collpoint, 3: free mass
        if (console::parse_x_arguments(argc,argv,"--solve-stages",solveStages) > 0) {
            weights.solveStages = 0x00;
            for (int const stage : solveStages) {
                weights.solveStages |= 1<<(stage);
            }
        }
        // check
        std::cout << "[" << __func__ << "] " << "solve Stages: " << std::bitset<sizeof(int)*8>(weights.solveStages) << std::endl;
        if (weights.solveStages & Weights::SOLVE_MOMENTA)
            std::cout << "[" << __func__ << "] " << "will solve stage0: momenta" << std::endl;
        if (weights.solveStages & Weights::SOLVE_COUPLED)
            std::cout << "[" << __func__ << "] " << "will solve stage1: coupled" << std::endl;
        if (weights.solveStages & Weights::SOLVE_FREE_CP)
            std::cout << "[" << __func__ << "] " << "will solve stage2: free collision point" << std::endl;
    }

    // anim2
    if (console::find_switch(argc,argv,"--anim2")) {
        tracking::io::readCeresOptions(argc,argv,weights);
        CeresVector3 maxSize(0.,0.,0.);
        for ( auto const& cuboid : cuboids ) {
            maxSize(0) = std::max(maxSize(0), std::abs(static_cast<CeresScalar>(cuboid.getSize()(0))));
            maxSize(1) = std::max(maxSize(1), std::abs(static_cast<CeresScalar>(cuboid.getSize()(1))));
            maxSize(2) = std::max(maxSize(2), std::abs(static_cast<CeresScalar>(cuboid.getSize()(2))));
        }
        std::cout << "[" << __func__ << "] cuboid maxSize: " << maxSize.transpose() << std::endl;
        weights.bounds.objectSize.lower = -maxSize/2.;
        weights.bounds.objectSize.upper =  maxSize/2.;
        return anim2(cuboids,frameIds,mapper,weights,rgbs,argc,argv);
    }

    if (!console::parse_arg(argc, argv, "--integral-steps", weights.poseIntegralSteps)) {
        std::cerr << "[" << __func__ << "] " << "Need flag --integral-steps (0.5..2) for safety reasons..." << std::endl;
        return EXIT_FAILURE;
    }

    // tracks
    console::parse_arg( argc,argv,"--track-dist", optParams.trackDistThresh );

    // tracks2D
    std::string                 inPath;
    tracking::GroupedTracks2d   tracks2d;
    bool                        hasInTracks;
    GroupsCuboidsT              groupsCuboids = { {1,0}, {2,1}, {3,1}, {4,1}, {5,1} };
    if ( (hasInTracks = console::parse_arg(argc,argv,"--dat",inPath)) ) {
        //tracking::BroxIO::readTracks( inPath, tracks2d, true );
        tracking::io::readSegmentedTracks(inPath,tracks2d);
        // http://www.peterkovesi.com/matlabfns/index.html#projective

        tracking::GroupedTracks2d longest2d;
        int kTracks(0);
        if ( console::parse_arg(argc,argv,"--longest",kTracks) )
        {
            tracking::Tracks2D::getLongestTracks( tracks2d, kTracks, longest2d, optParams.trackDistThresh );
            tracks2d = longest2d;
        }
        std::cout << "[" << __func__ << "] " << "have " << tracks2d.size() << " tracks in " << tracks2d.getGroupCount() << std::endl;
    }

    // animate
//    if ( console::find_switch(argc,argv,"--animate") ) {
    using namespace tracking;
    bool hasInfMass = false;

    // prune cuboids
    for ( auto& cuboid : cuboids ) {
        if ( !cuboid.isMassFinite() )
            hasInfMass = true;
        for ( auto it = cuboid.getStates().begin(); it != cuboid.getStates().end(); ) {
            if ( it->first < frameIds.front() || it->first > frameIds.back() )
                it = cuboid.removeState( it );
            else
                ++it;
        }
    }

    tracking::bundle_physics::BundleWithPhysicsResult initial;

    std::string initPath;
    if ( console::parse_arg(argc,argv,"--init",initPath) ) {
//        weights.initFlags |= bundle_physics::Weights::INIT_FLAGS::USE_INPUT_TRANSLATIONS;
        weights.initFlags |= bundle_physics::Weights::INIT_FLAGS::USE_INPUT_PARABOLAS;
        if (EXIT_SUCCESS != bundle_physics::io::readInitialization(initial, initPath)) {
            std::cout << "[" << __func__ << "] " << "could not read init..." << std::endl;
            return EXIT_FAILURE;
        }
        if (console::find_switch(argc,argv,"--fix-parabolas")) {
            weights.fixFlags |= bundle_physics::Weights::FIX_FLAGS::FIX_PARABOLAS;
        }
        if (initial.collTimes.size()) {
            // make sure, it's not an integer (confuses output)
            for (auto &collTime : initial.collTimes) {
                if ((collTime.second - std::floor(collTime.second)) < kSmallDiff)
                    collTime.second = std::floor(collTime.second) + 0.02;
            }
            std::cout << "[" << __func__ << "] " << "fixing collTime to" << initial.collTimes.at(0) << std::endl;
            weights.initFlags |= bundle_physics::Weights::USE_INPUT_COLLTIMES;
//            weights.fixFlags |= bundle_physics::Weights::FIX_COLLTIMES;
            if (std::abs(std::floor(initial.collTimes.at(0)) - static_cast<float>(frameIds.at(1))) > kSmallDiff) {
                frameIds.at(1) = static_cast<FrameId>(std::floor(initial.collTimes.at(0)));
                std::cout << "[" << __func__ << "] " << "middle FrameId is now: " << frameIds.at(1) << std::endl;
            }
        }
    } //... --init

    std::string showFlags = "11111";
    console::parse_arg(argc,argv,"--show-flags",showFlags);

    // input collision time
    std::vector<Scalar> collTimes;
    if (console::parse_x_arguments(argc,argv,"--coll-times", collTimes) >= 0) {
        if (initial.collTimes.size()) {
            std::cerr << "[" << __func__ << "] " << "overriding init colltime with --coll-times" << std::endl;
            initial.collTimes.clear();
        }
        weights.initFlags |= bundle_physics::Weights::USE_INPUT_COLLTIMES;
        weights.fixFlags |= bundle_physics::Weights::FIX_COLLTIMES;
        for ( size_t i = 0; i != collTimes.size(); ++i ) {
            initial.collTimes.insert(std::make_pair(Scalar(i), collTimes[i]));
            frameIds.at(i+1) = static_cast<FrameId>(std::floor(collTimes[i]));
        }
    } else {
        weights.bounds.collTime.lower = static_cast<double>(frameIds.at(1)) - (weights.fps / 60. - 1.);
        weights.bounds.collTime.upper = static_cast<double>(getNext(frameIds.at(1))) + (weights.fps / 60. - 1.);
    }

    // --2dParabolas
//    bool useParabola2dTerm = console::find_switch(argc,argv,"--para2d");
//    if (useParabola2dTerm)
//        std::cout << "[" << __func__ << "] " << "use parabola2dTerm: " << std::boolalpha << useParabola2dTerm << std::endl;

    // Curtain1: momenta
    if (0) {
        initial.momenta[1][0] = { 0.0660085,  0.0333842 , 0.217801};
        initial.momenta[1][1] = { -0.0228626,  -0.148693,   0.107043};
        // duck:
        initial.momenta[0][0] = { 0.01, 0.01, -0.0332678 };
        initial.momenta[0][1] = { 0.0316294,   0.12934, 0.0676543 }; // looked veryvery good  0.0316294   0.12934 0.0676543

        weights.initFlags |= Weights::USE_INPUT_MOMENTUM;
//        weights.fixFlags |= Weights::FIX_MOMENTA;
//        initial.mass = 0.393/0.237;
//        weights.initFlags |= Weights::USE_INPUT_MASS;
    }

    // run
    using namespace tracking::bundle_physics;
    BundleWithPhysicsResult result;
    ret = EXIT_SUCCESS;

    if (cuboids.size() > 2) {
        throw new std::logic_error("Triple collision experimental...");
        //tripleColl(result, cuboids, tracks2d, rgbs, frameIds, mapper, weights, &initial, showFlags);
    } else {
        if (hasInfMass)
            ret = infMass(result, cuboids, tracks2d, rgbs, frameIds, mapper, weights, &initial, showFlags);
        else {
            bool useParabola2dTerm = console::find_switch(argc, argv, "--para2d");
            if (useParabola2dTerm)
                std::cout << "[" << __func__ << "] " << "use parabola2dTerm: " << std::boolalpha << useParabola2dTerm
                          << std::endl;
            ret = BundleWithPhysics::animateCuboids2(result, cuboids, tracks2d, rgbs,
                                                     frameIds, mapper, weights, &initial,
                                                     useParabola2dTerm, showFlags);
        }
    }
    if (ret != EXIT_SUCCESS)
        return ret;

    if (result.cuboids && result.cuboids->size()) {
        std::cout << "animateCuboids returned " << result.cuboids->size() << " cuboids with " << result.cuboids->at(0).getStates().size() << " states" << std::endl;
        tracking::bundle_physics::io::writeInitialization(result, "result.json", weights.fps);

        std::string basis("");
        if ( cuboidsPaths.size() )
            basis = "multipleCuboids.json";
        else
            basis = cuboidsPaths.at(0);
        std::cout << "path: " << basis;
        size_t lastSlash = basis.rfind('/');
        std::string dir = ".";
        if ( lastSlash != std::string::npos )
            dir = basis.substr( 0, lastSlash );
        std::cout << ", dir : " << dir << std::endl;
        size_t lastDot = basis.rfind('.');
        std::string stem = basis.substr( lastSlash + 1, lastDot );
        if ( stem.find("Cuboids") )
            stem = stem.substr( 0, stem.find("Cuboids") );
        else if ( stem.find("Cuboid") )
            stem = stem.substr( 0, stem.find("Cuboid") );
        else if ( stem.find("cuboids") )
            stem = stem.substr( 0, stem.find("cuboids") );
        else if ( stem.find("cuboid") )
            stem = stem.substr( 0, stem.find("cuboid") );
        std::cout << "stem:" << stem << std::endl;
//        if ( result.cuboids->size() > 1 )
//            std::cerr << "unprepared for more, than one cuboid" << std::endl;
//        else
//            result.cuboids->at(0).setName( stem );
        tracking::bundle_physics::io::writeCuboids( *result.cuboids, dir + "/" + stem + "Traj" + std::to_string(frameIds.front()) + "_" + std::to_string(frameIds.back()) + ".json", result.collisions.at(0).cor );
    }

    std::string evalPath("eval.log");
    if (console::parse_arg(argc,argv,"--eval",evalPath)) {
        result.getLog().dump(evalPath, result, weights);
    }

    if (EXIT_SUCCESS == ret && weights.doVis && showFlags[0] == '1') {
        createMovie( *result.cuboids, rgbs, frameIds, mapper, &cuboids, result.tracks3d.get(), &tracks2d, &result.gravity, weights.fps );
    }

    return ret;
#if 0
    } //...animate

    if ( console::find_switch(argc,argv,"--5points") )
    {
#if TV_WITH_BUNDLER
        return tracking::test5Points( tracks2d, frameIds, mapper );
#else
        std::cout << "[" << __func__ << "]: re-enable WITH_BUNDLER flag in cmakelists to run --5points. " << std::endl;
        return EXIT_FAILURE;
#endif
    }

    // annotate tracks
    {
        tracking::Annotator annotator( "annotate", rgbs, "annotated.bin",
                                       mapper,
                                       frameIds,
                                       hasInTracks ? &tracks2d : nullptr
                                     );
        if ( !cuboids.size() )
        {
            //annotator.addCuboid( tracking::Cuboid(Vector3(0.305,0.228,0.183)) );
            annotator.addCuboid( tracking::Cuboid(Vector3(0.583,0.145,0.138)) );
        }
        else
            for ( tracking::Cuboid const& cuboid : cuboids )
                annotator.addCuboid( cuboid );
        //annotator.addCuboid( tracking::Cuboid(Vector3(5.,3.,0.1)) ); // wall

        //pose:
        // 0.761088 -0.289315 -0.580534
        // 0.447798 -0.413076  0.792918
        //-0.469255 -0.863445 -0.184811
        //pos:  0.247713 -0.518092      2.25
        //scale:
        // 0.30195        0        0        0
        //       0 0.273121        0        0
        //       0        0  0.22143        0
        //       0        0        0        1

        // wall:
//        pose:
//            0.866026  8.78967e-09     0.499999
//        -6.51217e-09            1 -6.29994e-09
//           -0.499999  2.19982e-09     0.866026
//        pos:    1.81674 -0.0920504       3.65
//        scale:
//          5   0   0   0
//          0   3   0   0
//          0   0 0.1   0
//          0   0   0   1

        char c(0);
        while ( c != 27 )
        {
            annotator.show();
            c = cv::waitKey();
            annotator.keyPressed( c );
        }
    } //...annotate
#endif
    return EXIT_SUCCESS;
} //...main()



