//
// Created by bontius on 28/03/16.
//

#include <tracking/phys/io/cuboidIo.h>
#include <tracking/phys/io/parabolaIo.h>
#include "tracking/phys/initialize/fitParabolas.h"
#include "tracking/phys/ceresUtil.h"
#include "tracking/phys/initialize/initialize.h"
#include "tracking/phys/initialize/pointTerm.h"
#include "tracking/phys/initialize/assignments.h"
#include "tracking/phys/initialize/bgFgExtractor.h"
#include "tracking/phys/initialize/parabola2d.h"
#include "tracking/phys/energyTerms/conservationTerm.h"
#include "tracking/phys/energyTerms/sfmImpulseTerm.h"
#include "tracking/phys/energyTerms/pointTerm.h"
#include "tracking/phys/physOutput.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/physFunctorInfo.h"
#include "tracking/phys/physIndexer.h"
#include "tracking/phys/consts.h"
#include "tracking/phys/weights.h"
#include "tracking/phys/partUtil.h"
#include "tracking/phys/vis/visPhys.h"
#include "tracking/phys/typedefsGeometry.h"
#include "tracking/phys/typedefs.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/mapper.h"
#include "tracking/common/assignments.h"
#include "tracking/common/util/energyPlotter.h"
#include "tracking/common/util/impl/parse.hpp"
#include "tracking/common/util/util.h"
#include "tracking/common/io/segmentedTracksIo.h"
#include "tracking/common/io/assignmentsIo.h"
#include "tracking/common/io/savePly.h"
#include "tracking/common/typedefs.h"

namespace tracking {
  namespace bundle_physics {
    enum CERES_STATE { VARIABLE, CONSTANT };
    extern void saveBlobs(BundleWithPhysicsResult const& init, GroupedTracks2d const& selectedTracks2d, TracksToCuboidsT const& assignments, RgbsT const& indexedRgbs, FrameIdsT const& frameIds, Mapper const& mapper);
    extern void saveBlobs2(Eigen::Vector2f const& roi, BundleWithPhysicsResult const& init, RgbsT const& indexedRgbs, FrameIdsT const& frameIds, Mapper const& mapper, cv::Mat const* bg = nullptr);
    extern BundleWithPhysicsResult fitParabola(Parabola2ds const& paths, FrameIdsT const& frameIds, Mapper const& mapper, Weights const& weights, const int nCuboids, RgbsT const& rgbs, bool const show);

    void setPointStates(CERES_STATE const state, ceres::Problem& problem, PhysIndexer& indexer, PhysFunctorInfosT const& functorInfos) {
        for (std::shared_ptr<PhysFunctorInfo> const& functorInfo : functorInfos) {
            std::shared_ptr<TrackPhysFunctorInfo> tpf = std::dynamic_pointer_cast<TrackPhysFunctorInfo>(functorInfo);
            if (tpf) {
                for (TrackId const &trackId : tpf->getTrackIds()) {
                    if (CERES_STATE::VARIABLE == state)
                        problem.SetParameterBlockVariable(indexer.getPointAddress(trackId));
                    else if (CERES_STATE::CONSTANT == state)
                        problem.SetParameterBlockConstant(indexer.getPointAddress(trackId));
                    else
                        std::cerr << "[" << __func__ << "] unprepared for this state..." << state << std::endl;
                }
            }
        } //...all functors
    } //...fixPoints()

    void setParabolaTranslationStates(CERES_STATE const state, ceres::Problem& problem, PhysIndexer& indexer, std::vector<CuboidId> const& participants, CollId const collId) {
        for (CuboidId const cuboidId : participants) {
            if (CERES_STATE::VARIABLE == state)
                problem.SetParameterBlockVariable(indexer.getParabolaTranslation(cuboidId, collId));
            else if (CERES_STATE::CONSTANT == state)
                problem.SetParameterBlockConstant(indexer.getParabolaTranslation(cuboidId, collId));
            else
                std::cerr << "[" << __func__ << "] unprepared for this state..." << state << std::endl;
        }
    } //...setParabolaTranslationStates()

    void setParabolaFreeStates(CERES_STATE const state, ceres::Problem& problem, PhysIndexer& indexer,std::vector<CuboidId> const& participants, CollId const collId) {
        for (CuboidId const cuboidId : participants) {
            if (CERES_STATE::VARIABLE == state)
                problem.SetParameterBlockConstant(indexer.getParabolaFreeParams(cuboidId, getPartIdBefore(collId)));
            else if (CERES_STATE::CONSTANT == state)
                problem.SetParameterBlockConstant(indexer.getParabolaTranslation(cuboidId, getPartIdAfter(collId)));
            else
                std::cerr << "[" << __func__ << "] unprepared for this state..." << state << std::endl;
        } //...for participants
    } //...setParabolaFreeStates()

    std::map<TrackId,Eigen::Vector3f> getTrackColors(RgbsT rgbs, GroupedTracks2d const tracks2d, GroupedTracks3d const& tracks3d)
    {
        std::map<TrackId,Eigen::Vector3f> colors;
        for (Track3D const& track3d : tracks3d) {
            TrackId const trackId = track3d.getTrackId();
            Track2D const& track2d = tracks2d.getTrackByLabel(trackId);
            int cnt(0);
            Eigen::Vector3f color(Eigen::Vector3f::Zero());
            for (auto const& pair : track2d.getPoints()) {
                FrameId const& frameId = pair.first;
                if (rgbs.find(frameId)==rgbs.end())
                    continue;
                cv::Mat const& rgb = rgbs.at(frameId);
                auto const& pnt = pair.second.getPoint();
                cv::Vec3b const& currColor = rgb.at<cv::Vec3b>(pnt(0),pnt(1));
                color(0) += currColor[0];
                color(1) += currColor[1];
                color(2) += currColor[2];
                ++cnt;
            }
            if (cnt)
                colors[trackId] = color/static_cast<float>(cnt);
            else
                colors[trackId] = color;
        }

        return colors;
    } //...getTrackColors()

    PhysIndexer allocate(GroupedTracks2d const &tracks2d, FrameIdsT const &frameIds, int nCuboids) {
        PhysIndexer indexer(frameIds,nCuboids);
        for (auto const &track2d : tracks2d)
            indexer.addPoint(track2d.getLabel());
        indexer.allocate();
        return indexer;
    }

    std::unique_ptr<ceres::Problem> setup(PhysIndexer &indexer, PhysFunctorInfosT &costFunctors, GroupedTracks2d const& tracks2d, CuboidsT const& cuboids, TracksToCuboidsT const& assignments,
        Mapper const& mapper, FrameIdsT const& frameIds, Weights const& weights, Consts const& consts, std::vector<CuboidId> const& /*participants*/) {

        std::unique_ptr<ceres::Problem> problem(new ceres::Problem);
        addPointTerms(*problem,indexer,costFunctors,assignments,tracks2d,cuboids,mapper,frameIds,weights,consts);
        //addBatchPointTerms(*problem,indexer,costFunctors,assignments,tracks2d,cuboids,mapper,frameIds,weights,consts,pointLoss);

        //addImpulseTerms(*problem, indexer, costFunctors, frameIds, weights, cuboids, participants, consts);
        //initCollPoint(problem, indexer, weights, participants, consts, initial);
        //if (weights.conservationWeight > 0.)
        //    sfmAddConservationTerms(*problem, indexer, costFunctors, frameIds, weights, cuboids, participants, consts );

//        if ( weights.gravityDownWeight > 0. )
//            addGravityTerm( problem, indexer, costFunctors, weights, consts );

        return problem;
    } //...setup()

    extern std::set<TrackId> solve(ceres::Problem &problem, PhysIndexer const& indexer, Weights const& weights, PhysFunctorInfosT &costFunctors);

    GroupedTracks3d filter(GroupedTracks3d const& groupedTracks3d, std::function<bool (Track3D const&)> selector)
    {
        GroupedTracks3d out;
        for (auto const &groupIdAndLinIds : groupedTracks3d.getGroups()) {
            GroupId const& groupId = groupIdAndLinIds.first;
            auto const& linIds = groupIdAndLinIds.second;
            for (LinId const &linId : linIds) {
                Track3D const &track3d = groupedTracks3d.getTrack(linId);
                //TrackId const &trackId = track3d.getTrackId();
                if (selector(track3d))
                    out.addTrack(track3d,groupId);
            }
        }

        return out;
    } //...filter()

    /** \brief Tracked points based physics (does not work), and ransac 2.5d parabola fitting for initialization. */
    int anim2(CuboidsT const& cuboids, FrameIdsT const& frameIds, Mapper const& mapper, Weights weights, LinRgbsT const& rgbs, int argc, const char** argv) {

        int ret = EXIT_SUCCESS;

        RgbsT indexedRgbs;
        {
            LinId linId(0);
            for (FrameId frameId = frameIds.front(); frameId <= frameIds.back(); step(frameId),++linId)
                indexedRgbs[frameId] = rgbs.at(linId);
        }

        // --bg
        cv::Mat bg;
        {
            std::string bgPath("");
            if (!console::parse_arg(argc, argv, "--bg", bgPath) && !console::parse_arg(argc,argv,"--back",bgPath)) {
                std::cout << "[" << __func__ << "] " << "not using background" << std::endl;
            } else {
                bg = cv::imread(bgPath,cv::IMREAD_COLOR);
                if (bg.empty()) {
                    std::cerr << "[" << __func__ << "] " << "could not read " << bgPath << std::endl;
                    throw new std::runtime_error("");
                }
            }
        }

        // --bbox
        Eigen::Vector2f clipSize(0.,0.);
        {
            std::vector<float> clips;
            if (console::parse_x_arguments(argc,argv,"--bbox",clips)) {
                if (clips.size() == 2)
                    clipSize << clips[0], clips[1];
                else if (clips.size() == 1)
                    clipSize << clips[0], clips[0];
                else {
                    std::cerr << "[" << __func__ << "] " << "can't interpret bbox with " << clips.size() << " elements. Need 1 or 2." << std::endl;
                    return 1;
                }
            }
        }
        cv::Size bBox { static_cast<int>(std::round(clipSize(0))),
                        static_cast<int>(std::round(clipSize(1))) };

        // --crop
        std::array<int,4> crops;
        {
            crops.fill(0);
            std::vector<int> _crops;
            console::parse_x_arguments(argc,argv,"--crop",_crops);
            std::copy(_crops.begin(), _crops.end(), crops.begin());
        }

        // --lookaround
        int lookAround = 1;
        console::parse_arg(argc,argv,"--lookaround",lookAround);
        std::cout << "[" << __func__ << "] " << "setting lookaround to " << lookAround << std::endl;

        // --silent
        bool silent = console::find_switch(argc,argv,"--silent");
        // --mask
        std::string maskPath("");
        console::parse_arg(argc,argv,"--mask",maskPath);

        // --half-spread-pos
        float halfSpreadPos = 0.;
        console::parse_arg(argc,argv,"--half-spread-pos",halfSpreadPos);
        std::cout << "[" << __func__ << "] " << "HalfSpreadPos: " << halfSpreadPos << std::endl;

        // --erode (iterations)
        int erodeIterations(2), closeIterations(6), morphSize(3);
        console::parse_arg(argc,argv,"--erode",erodeIterations);
        std::cout << "[" << __func__ << "] " << "setting erode iterations to " << erodeIterations << std::endl;

        // --close (iterations)
        console::parse_arg(argc,argv,"--close",closeIterations);
        std::cout << "[" << __func__ << "] " << "setting close iterations to " << closeIterations << std::endl;

        // --morph-size (2k+1...so 3..7)
        console::parse_arg(argc,argv,"--morph-size",morphSize);
        std::cout << "[" << __func__ << "] " << "setting morph-size to " << morphSize << std::endl;

        // --minarea
        int minBlobArea(1500);
        console::parse_arg(argc,argv,"--min-area",minBlobArea);
        std::cout << "[" << __func__ << "] " << "setting minBlobArea to " << minBlobArea << std::endl;

        // --ignore
        int ignoreAroundCollTime = 2;
        console::parse_arg(argc,argv,"--ignore",ignoreAroundCollTime);
        std::cout << "[" << __func__ << "] " << "Ignoring blob centroids " << ignoreAroundCollTime << " before and after collTime" << std::endl;

        // --learn-rate
        float learningRate = 0.01;
        console::parse_arg(argc,argv,"--learn-rate", learningRate);
        std::cout << "[" << __func__ << "] " << "BGFG learningRate " << learningRate << std::endl;

        // --fix-time
        if (console::find_switch(argc,argv,"--fix-time"))
            weights.fixFlags |= Weights::FIX_COLLTIMES;

        // --flat-limit
        float flatLimit = 0.f;
        console::parse_arg(argc,argv,"--flat-limit",flatLimit);
        std::cout << "[" << __func__ << "] " << "Limiting free rotation angle to " << flatLimit << " degrees" << std::endl;

        // --weight-size
        if (console::parse_arg(argc,argv,"--weight-size",weights.sizePriorWeight)) {
            std::cout << "[" << __func__ << "] " << "Size-prior weight is " << weights.sizePriorWeight << std::endl;
        }

        // --weight-repulse
        if (console::parse_arg(argc,argv,"--weight-repulse", weights.repulseWeight)) {
            std::cout << "[" << __func__ << "] " << "Repulse weight is " << weights.repulseWeight << std::endl;
        }


        // --raw-centroids
        bool const saveRawCentroids = console::find_switch(argc,argv,"--raw-centroids");
        if (!saveRawCentroids) {
            std::cerr << "[" << __func__ << "] " << "don't forget --raw-centroids" << std::endl;
            throw new std::runtime_error("");
        }

        // --inlier-div
        float inlierDiv = 2.5;
        if (console::parse_arg(argc,argv,"--inlier-div",inlierDiv)) {
            std::cout << "[" << __func__ << "] " << "Inlier distance division by " << inlierDiv << std::endl;
        }

        srand(time(NULL));

        // fit only to finite cuboids
        CuboidsT finite(cuboids.size());
        {
            auto it = std::copy_if(cuboids.begin(), cuboids.end(), finite.begin(), [](Cuboid const &cuboid) { return cuboid.isMassFinite(); });
            finite.resize(std::distance(finite.begin(), it));
        }

        // WORK
        const int64 start = cv::getTickCount();
        BundleWithPhysicsResult init = ransacParabolas(frameIds, mapper, weights, finite, indexedRgbs, !silent, bBox, &crops, erodeIterations, closeIterations, morphSize, minBlobArea, bg.empty()
                                                                                                                                                                                        ? nullptr : &bg
                                                       , ignoreAroundCollTime, lookAround, halfSpreadPos, flatLimit, maskPath, learningRate, saveRawCentroids, inlierDiv);
        const double timeSec = (cv::getTickCount() - start) / cv::getTickFrequency();
        std::cout << "[bgfg+ransac] " << timeSec << " sec" << std::endl;

        #warning: check if this holds for infmass as well:
        // copy input poses to output (we only inited positions)
        for (auto& outCuboid : *init.cuboids) {
            if (outCuboid.isMassFinite())
                continue;
            std::cout << "[" << __func__ << "] " << "outName: " << outCuboid.getName() << std::endl;
            auto iter = std::find_if(std::begin(cuboids), std::end(cuboids), [&outCuboid](Cuboid const &cuboid) { return cuboid.getName() == outCuboid.getName();});
            if (iter != cuboids.end()) {
                std::cout << "[" << __func__ << "] " << "adding poses to " << outCuboid.getName() << " from " << iter->getName() << std::endl;
                std::for_each(std::begin(iter->getStates()), std::end(iter->getStates()), [&outCuboid](std::pair<FrameId,PoseLoc> const &frameIdState) {
                    if (frameIdState.second.hasPose())
                        outCuboid.setPose(frameIdState.first, frameIdState.second.getPose());
                });
            } //...if found input cuboid with same name
        } //...for outcuboids

        // replace cuboid positions with circles...

        // check order
        if (finite.size() == cuboids.size()) {
            // swap, if parabolas not in order of: "from left, from right"
            if (init.parabolas.begin()->second.at(0).s > 0.) {
                std::cout << "[" << __func__ << "] " << "swapping" << std::endl;
                // parabolas
                std::swap(init.parabolas.at(0), init.parabolas.at(1));
                // collisions (todo)
                if (init.collisions.size()) {
                    std::cerr << "[" << __func__ << "] " << "TODO: implement swap of collision states!" << std::endl;
                    throw new std::runtime_error("");
                }
                // cuboids
                std::swap(init.cuboids->at(0), init.cuboids->at(1));
                // cubiodId aka objId
                auto const tmpId = init.cuboids->at(0).getObjId();
                init.cuboids->at(0).setObjId(init.cuboids->at(1).getObjId());
                init.cuboids->at(1).setObjId(tmpId);
                // cuboid name (Blender readin uses it!)
                auto const tmpName = init.cuboids->at(0).getName();
                init.cuboids->at(0).setName(init.cuboids->at(1).getName());
                init.cuboids->at(1).setName(tmpName);
                auto const tmpSize = init.cuboids->at(0).getSize();
                init.cuboids->at(0).getSize() = init.cuboids->at(1).getSize();
                init.cuboids->at(1).getSize() = tmpSize;

            } //...if swap needed
        } else { // if infMass
            if (cuboids.size() > 2) {
                std::cerr << "[" << __func__ << "] " << "unprepared for more than two cuboids and infinite mass...todo" << std::endl;
                throw new std::runtime_error("");
            }
            // copy input infMass to output
            for (auto const& inCuboid : cuboids) {
                std::cout << "[" << __func__ << "] " << "inName: " << inCuboid.getName() << std::endl;
                if (!inCuboid.isMassFinite())
                    init.cuboids->push_back(inCuboid);
            }
        } //...order cuboids


        // output cuboids
        tracking::bundle_physics::io::writeCuboids(*init.cuboids, "2dparabolas.json", 0.);

        // output initialization (parabola parameters)
        tracking::bundle_physics::io::writeInitialization(init,"2dparabolasInit.json", weights.fps);

        // output patches (for orientation matching)
        if (!console::find_switch(argc,argv,"--no-patches"))
            saveBlobs2(clipSize, init, indexedRgbs,frameIds,mapper, bg.empty() ? nullptr : &bg);

        std::cout << "[bgfg+ransac] " << timeSec << " sec" << std::endl;
        return ret;
#if 0
        // ==========================================================================================================
        // Brox tracks based orientations (doesn't work yet. Todo: add neighbourhood term instead of compactness)

        int const nCuboids = 2;
        std::vector<CuboidId> const participants = {0,1};
        CollId const collId = 0;

        size_t nTracksPerObj = 500;
        console::parse_arg(argc, argv, "--ntracks",nTracksPerObj);

        std::string datPath("");
        if (EXIT_SUCCESS == ret) {
            if (!console::parse_arg(argc, argv, "--dat", datPath)) {
                std::cerr << "[" << __func__ << "] " << "need \"--dat\" flag as input" << std::endl;
                ret = EXIT_FAILURE;
            }
        } //...read datPath

        GroupedTracks2d groupedTracks2d;
        if (EXIT_SUCCESS == ret) {
            ret = tracking::io::readSegmentedTracks(datPath, groupedTracks2d);
            if ( EXIT_SUCCESS == ret )
                std::cout << "[" << __func__ << "] " << "tracks read..." << std::endl;
        } //...read tracks

        TracksToCuboidsT assignments;
        if (EXIT_SUCCESS == ret) {
            std::string assignmentsPath("");
            if (!console::parse_arg(argc, argv, "--ass", assignmentsPath) || assignmentsPath.empty()) {
                std::cerr << "[" << __func__ << "] need --ass flag to start" << std::endl;
                ret = EXIT_FAILURE;
            } else {
                ret = tracking::io::readAssignments(assignments, assignmentsPath, groupedTracks2d,"cuboidId");
                if (EXIT_SUCCESS != ret) {
                    std::cerr << "[" << __func__ << "] read assignments failed" << std::endl;
                }
            }
        } //...read assignments

        GroupedTracks2d selectedTracks2d;
        if (groupedTracks2d.size())
        {
            EnergyPlotter lengths;
            int target = nTracksPerObj;
            for (auto const &groupIdAndLinIds : groupedTracks2d.getGroups()) {
                GroupId const& groupId = groupIdAndLinIds.first;
                auto const& linIds = groupIdAndLinIds.second;
                float const chance = float(target)/float(linIds.size());
                int skipCount = 0;
                for (LinId const &linId : groupIdAndLinIds.second) {
                    Track2D const &track2D = groupedTracks2d.getTrack(linId);
                    TrackId const &trackId = track2D.getTrackId();
                    if (assignments.find(trackId) != assignments.end()) {
                        if (track2D.size() < 5) {
                            ++skipCount;
                            continue;
                        }
                        double length = track2D.getLength();
                        lengths.addValue(length,"Lengths");
                        if ( track2D.getLength() < 5) {
                            continue;
                        }
                        //if ( track2D.getPoints().rbegin()->first < frameIds.at(1))
                        //    continue;
                        if (randf() > chance)
                            continue;
                        selectedTracks2d.addTrack(track2D,groupId);
                    }
                }
                if (skipCount)
                    std::cout << "skipped " << skipCount << " tracks based on obsCount" << std::endl;
            }
            lengths.plot();
            std::cout << "[" << __func__ << "] working with " << selectedTracks2d.size() << " tracks" << std::endl;
        }

        // almost worked on doodle7_sub:
        //     pointWeight: 4.   ,   pointHuber: 0.
        //   compactWeight: 0.005, compactHuber: 0.2
        weights.pointHuberParam       = 0.;
        weights.compactnessHuberParam = 0.;
        weights.compactnessWeight     = .001;
        weights.poseIntegralSteps     = .1;
        weights.fixFlags              |= Weights::FIX_GRAVITY;

        Consts const consts(weights.fps,frameIds);
        PhysFunctorInfosT               costFunctors;
        PhysIndexer                     indexer      = allocate(selectedTracks2d, frameIds, nCuboids);
        std::unique_ptr<ceres::Problem> problem      = setup   (indexer,costFunctors,selectedTracks2d,cuboids,assignments,mapper,frameIds,weights,consts,participants);

        initialize(*problem,indexer,costFunctors,frameIds, selectedTracks2d, assignments,weights,consts,mapper,nCuboids,participants,collId,&init);

        // fix global rotation
        indexer.getParabolaRotationShared()[0] = 0.;
        indexer.getParabolaRotationShared()[1] = 0.;
        problem->SetParameterBlockConstant(indexer.getParabolaRotationShared()); // x, y1

        // fix translation
        setParabolaTranslationStates(CERES_STATE::CONSTANT,*problem,indexer,participants,collId);

        // fix local rotation and shape
        setParabolaFreeStates       (CERES_STATE::CONSTANT,*problem,indexer,participants,collId);

        // optimize
        std::set<TrackId> inliers = solve(*problem,indexer,weights,costFunctors);

        // report
        std::cout << "[0][0] y0: " << indexer.getParabolaFreeParams (0,0)[0] << std::endl;
        std::cout << "[0][1] y0: " << indexer.getParabolaFreeParams (0,1)[0] << std::endl;
        std::cout << "[1][0] y0: " << indexer.getParabolaFreeParams (1,0)[0] << std::endl;
        std::cout << "[1][1] y0: " << indexer.getParabolaFreeParams (1,1)[0] << std::endl;

        // fix points
        setPointStates(CERES_STATE::CONSTANT,*problem,indexer,costFunctors);

#if 1   // add physics
        // conservation
        sfmAddConservationTerms(*problem, indexer, costFunctors, frameIds, weights, cuboids, participants, consts );
        // impules
        addImpulseTerms(*problem, indexer, costFunctors, frameIds, weights, cuboids, participants, consts);
        // collision point
        initCollPoint2(*problem,indexer,weights,participants,consts,nullptr);
        std::cout << "releasing collision point" << std::endl;
        problem->SetParameterBlockVariable( const_cast<ceres::CeresScalar*>(indexer.getCollisionPointConst(0)) );

        // optimize
        inliers = solve(*problem,indexer,weights,costFunctors);
#endif

        // free translation
        setParabolaTranslationStates(CERES_STATE::VARIABLE,*problem,indexer,participants,collId);
        // optimize
        inliers = solve(*problem,indexer,weights,costFunctors);

        // fix parabolas, reweight points
//        problem->SetParameterBlockConstant(indexer.getParabolaFreeParams(0,0));
//        problem->SetParameterBlockConstant(indexer.getParabolaFreeParams(0,1));
//        problem->SetParameterBlockConstant(indexer.getParabolaFreeParams(1,0));
//        problem->SetParameterBlockConstant(indexer.getParabolaFreeParams(1,1));
//        auto const& pointWrapperInfo = indexer.getWrapperInfo(POINT_TERM);
//        pointWrapperInfo.lossWrapper->Reset(chooseHuberOrTrivial(pointWrapperInfo.weight * 5.,0.),ceres::TAKE_OWNERSHIP);
//        auto const& compactnessWrapperInfo = indexer.getWrapperInfo(COMPACTNESS_TERM);
//        compactnessWrapperInfo.lossWrapper->Reset(chooseHuberOrTrivial(0.,0.),ceres::TAKE_OWNERSHIP);
//        inliers = solve(*problem,indexer,weights,costFunctors);

        // output
        BundleWithPhysicsResult out;
        output(out, cuboids, selectedTracks2d, frameIds, indexer, consts, weights.doVis, &assignments);

        // check height
        {
            for (auto const& cuboid : *out.cuboids) {
                struct TimedPoint { FrameId frameId; Eigen::Vector3f pos, linVel; };
                TimedPoint mn{INVALID_FRAME_ID,Eigen::Vector3f{1.e6f,1.e6f,1.e6f},Eigen::Vector3f{0.f,0.f,0.f}},
                           mx{INVALID_FRAME_ID,Eigen::Vector3f{-1.e6f,-1.e6f,-1.e6f},Eigen::Vector3f{0.f,0.f,0.f}};
                for (auto const& frameIdAndState : cuboid.getStates()) {
                    if ( frameIdAndState.second.getPosition()(1) < mn.pos(1) ) {
                        mn.frameId = frameIdAndState.first;
                        mn.pos = frameIdAndState.second.getPosition();
                        mn.linVel = frameIdAndState.second.getLinVel();
                    }
                    if ( frameIdAndState.second.getPosition()(1) > mx.pos(1) ) {
                        mx.frameId = frameIdAndState.first;
                        mx.pos = frameIdAndState.second.getPosition();
                        mx.linVel = frameIdAndState.second.getLinVel();
                    }
                }
                std::cout << "minpos: " << mn.pos.transpose() << " [" << mn.frameId << "]" << ", linVel: " << mn.linVel.transpose() << std::endl;
                std::cout << "maxpos: " << mx.pos.transpose() << " [" << mx.frameId << "]" << ", linVel: " << mx.linVel.transpose() << std::endl;
            }
        }

        if (inliers.size())
            *out.tracks3d = filter(*out.tracks3d,[&inliers](Track3D const& track3d){ return (inliers.find(track3d.getTrackId())!=inliers.end());});
        std::map<TrackId,Eigen::Vector3f> colors = getTrackColors(indexedRgbs,selectedTracks2d,*out.tracks3d);
        tracking::io::savePly("c2.ply",*out.tracks3d, nullptr,&colors);
        showPhysOutput(out, cuboids, indexer, consts);
        createMovie(*out.cuboids, rgbs, frameIds, mapper, &cuboids, out.tracks3d.get(), &selectedTracks2d, &out.gravity );
        return ret;
#endif // Brox orientation

    } //...anim2()
  } //...ns bundle_phsyics
} //...ns tracking