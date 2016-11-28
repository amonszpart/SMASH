//
// Created by bontius on 01/05/16.
//
#include "tracking/phys/initialize/fitParabolas.h"
#include "tracking/phys/initialize/bgFgExtractor.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/partUtil.h"
#include "tracking/common/groupedTracks.h"
#include <tracking/common/util/util.h>
#include <tracking/common/util/energyPlotter.h>
#include "opencv2/xfeatures2d.hpp"

namespace tracking {
namespace bundle_physics {

/** \param[in] ignoreAroundCollTime how many frames around collision time should we not assign circles through. */
std::pair<Scalar, size_t>
assignCirclesToTracks(std::map<FrameId, CirclesByAreaT> const& circles, GroupedTracks3d const& tracks3d,
                      double const inlierThresh, float collTime, Mapper const& mapper,
                      Parabola2ds *out, int ignoreAroundCollTime, int const nCuboids) {
    Parabola2ds parabolas;

    std::pair<Scalar,size_t> score(0.,0);
    Scalar sumWeights = 0.f;
    for (auto const &frameIdCircles : circles) {
        FrameId const frameId = frameIdCircles.first;
        if (std::abs(static_cast<Scalar>(frameId) - collTime) < ignoreAroundCollTime)
            continue;
        std::vector<std::pair<Scalar,std::pair<TrackId, Circle> > > all;
        for (auto circleIter = frameIdCircles.second.rbegin();
             circleIter != frameIdCircles.second.rend(); ++circleIter) {
            auto const& cvPnt = circleIter->second.getCentroid();
            Eigen::Vector2f pnt(cvPnt.x, cvPnt.y);
            for (auto const& track : tracks3d) {
                TrackId const trackId = track.getTrackId();
                if (!track.hasPoint(frameId))
                    continue;

                auto const p2   = mapper.to2D(track.getPoint(frameId).getPoint());
                Scalar     dist = (p2.head<2>() - pnt).norm();
                if (dist < inlierThresh) {
                    //dists.insert(std::make_pair(dist, std::make_pair(trackId, circleIter->second)));
                    all.push_back(std::make_pair(dist, std::make_pair(trackId, circleIter->second)));
                }
            }
        }

        PartId const partId = (frameId < collTime) ? 0 : 1;
        std::random_shuffle(all.begin(),all.end());
        // check for one assignment each track
        std::set<TrackId> seen;
        for (auto const& distAndCircle : all) {
            TrackId const trackId = distAndCircle.second.first;
            if (seen.find(trackId) != seen.end())
                continue;
            CuboidId const cuboidId = trackId;
            if (cuboidId >= nCuboids)
                std::cerr << "[" << __func__ << "] " << "noooooooooo" << std::endl;
            seen.insert(trackId); // track is assigned to circle, can't be taken again...
            score.first += distAndCircle.first * distAndCircle.second.second.getArea();
            sumWeights += distAndCircle.second.second.getArea();
            ++score.second;
//                Eigen::Vector2f pnt(distAndCircle.second.second.getCentroid().x,distAndCircle.second.second.getCentroid().y);
//                parabolas[{cuboidId,partId}].track.addPoint(frameId,pnt);
            parabolas[{cuboidId,partId}].addCircle(frameId, /* circle: */ distAndCircle.second.second);
            if (seen.size() == tracks3d.size())
                break;
        }
    }
    if (score.second)
        score.first /= static_cast<Scalar>(sumWeights);

    if (out)
        *out = std::move(parabolas);
    return score;
}

float checkPatchSimilarity(Parabola2ds const& parabolas, RgbsT const& rgbs/*, cv::Size const bbox*/) {
    std::map<CuboidId, std::vector<cv::Mat> > descs;
    std::vector<Parabola2ds::key_type> keys;
    for (auto const& cuboidIdPartIdAndParabola : parabolas) {
       keys.push_back(cuboidIdPartIdAndParabola.first);
    }
//        for (auto const& cuboidIdPartIdAndParabola : parabolas) {
    std::cout << "[" << __func__ << "] " << "starting collections..." << std::endl;
#       pragma omp parallel for shared(descs)
    for (size_t i = 0; i < keys.size(); ++i) {
        cv::Ptr<cv::DescriptorExtractor> de = cv::xfeatures2d::SIFT::create(32);
//        cv::Ptr<cv::DescriptorExtractor> de = cv::xfeatures2d::SIFT::create(32);
        auto const key = keys.at(i);
//            std::cout << "[" << __func__ << "] " << "key ok" << std::endl;
        CuboidId cuboidId = key.first; //cuboidIdPartIdAndParabola.first.first;
//            PartId partId = cuboidIdPartIdAndParabola.first.second;
        Parabola2d const& parabola2d = parabolas.at(key); //cuboidIdPartIdAndParabola.second;
//            std::cout << "[" << __func__ << "] " << "parabolas.at(" << key.first << "," << key.second << ") ok" << std::endl;
        for (auto const& frameIdPoint : parabola2d.getTrack().getPoints()) {
            if (randf() > 30. / rgbs.size())
                continue;
            FrameId const frameId = frameIdPoint.first;
#if 0
            cv::Mat img = rgbs.at(frameId).clone();
            cv::Point2i const pnt(frameIdPoint.second(0),frameIdPoint.second(1));
            cv::Rect rect{ std::max(0,pnt.x-bbox.width/2),
                           std::max(0,pnt.y-bbox.height/2),
                           bbox.width,bbox.height};
            rect.width = std::min(img.cols - rect.x,bbox.width);
            rect.height = std::min(img.rows - rect.y,bbox.height);
            if (rect.width < 0 || rect.height < 0)
                continue;

#else
            cv::Mat const img = rgbs.at(frameId);
            cv::Mat mask(cv::Mat::zeros(img.rows,img.cols,CV_8UC1));
            std::vector<ContourT> hull(1);
            cv::convexHull(cv::Mat(parabola2d.getCircle(frameId).getContour()), hull[0], false);
            cv::drawContours(mask, hull, -1, cv::Scalar(255.), -1, 8, std::vector<cv::Vec4i>());
//                cv::Mat clip(cv::Mat::zeros(img.size(),img.type()));
//                img.copyTo(clip, mask);
            cv::Mat clip = img;
#endif
            std::vector<cv::KeyPoint> keypoints;
            cv::Mat descriptors;
            de->detectAndCompute(clip,mask,keypoints,descriptors);
#               pragma omp critical(PATCH_SIM_DESCS)
            {
                  descs[cuboidId].push_back(descriptors);
//                    cv::imshow("mask",mask);
//                    cv::waitKey(10);
            }
        }
    }

    std::cout << "[" << __func__ << "] " << "starting matching" << std::endl;
    cv::BFMatcher matcher;
    float score = 0.f;
    int i = 0;
#       pragma omp parallel shared(score,i) private(matcher)
    {
//            for (auto const &cuboidAndDescs : descs) {
        for (auto cuboidAndDescs = descs.begin(); cuboidAndDescs != descs.end(); ++cuboidAndDescs) {
#               pragma omp task
            {
                auto it0 = cuboidAndDescs->second.begin();
                for (auto it1 = std::next(it0); it1 != cuboidAndDescs->second.end(); ++it0, ++it1) {
                    std::vector<std::vector<cv::DMatch> > matches;
                    matcher.knnMatch(*it0, *it1, matches, /* k: */ 2);
                    for (auto const &match : matches) {
                        if (match.size() < 2)
                            continue;
                        if (match.at(0).distance < 0.8 * match.at(1).distance) {
#                               pragma omp critical (PATCH_SIM_SCORE)
                            {
                                score += match.at(0).distance;
                                ++i;
                            } //...omp critical
                        } //...if distance criterion
                    } //...for each match
                } //...pairwise
            } //...omp task
        } //...for descs
    } //...omp parallel
    std::cout << "[" << __func__ << "] " << "finished matching..." << std::endl;
//        if (!i)
    if (i < 2) {// changed on 160511
        std::cout << "[" << __func__ << "] " << "good sift: " << i << std::endl;
        return std::numeric_limits<float>::max();
    }
    else {
        std::cout << "[" << __func__ << "] " << "good sift: " << i << std::endl;
        return score / static_cast<float>(i);
    }
} //...checkPatchSimilarity

bool parabolasSimilar(BundleWithPhysicsResult const& result, int const nParabolas) {
    return (nParabolas > 2) &&
           (Parabola::similar(result.parabolas.at(0).at(0),result.parabolas.at(1).at(0)) ||
            Parabola::similar(result.parabolas.at(0).at(1),result.parabolas.at(1).at(1))   );
}

BundleWithPhysicsResult
ransacParabolas(FrameIdsT const& frameIds, Mapper const& mapper, Weights const& weights, CuboidsT const& cuboids,
                RgbsT const& rgbs, bool const debug, cv::Size const&/*bbox*/, std::array<int, 4>* const crops,
                int const erodeIterations, int const closeIterations, int const morphSize, int const minBlobArea,
                cv::Mat* const bg, int const ignoreAroundCollTime, int const lookAround, float const halfSpreadPos,
                float const flatLimit, std::string const maskPath, double learningRate, bool const saveRawCentroids,
                float const inlierDiv) {
    bool const detectShadows = true;
    int const nCuboids = std::count_if(cuboids.begin(),cuboids.end(),[](Cuboid const& cuboid){ return cuboid.isMassFinite();});
    int const nParabolas  = nCuboids *2;  // how many do we need
    std::cout << "[" << __func__ << "] " << "need " << nParabolas << " parabolas" << std::endl;
    if (static_cast<int>(cuboids.size()) != nCuboids) {
        std::cerr << "[" << __func__ << "] " << "Need exactly two cuboids as input (for their sizes)" << ", num is: " << cuboids.size() << std::endl;
        throw new std::runtime_error("");
    }
//        Scalar collTime = frameIds.at(1)+0.5;
    cv::Size const imSize = rgbs.begin()->second.size();
    cv::Point2f    imageCentroid(imSize.width / 2., imSize.height / 2.);

    cv::Mat mask;
    if (maskPath.size()) {
        mask = cv::imread(maskPath, cv::IMREAD_GRAYSCALE);
        if (mask.empty()) {
            std::cerr << "[" << __func__ << "] " << "could not read mask: " << maskPath << std::endl;
            throw new std::runtime_error("Could not read mask");
        }
    }

    int64 const startTime = cv::getTickCount();
    BgFgExtractor bgFgExtractor(64, erodeIterations, closeIterations, morphSize, detectShadows);
    if (bg && !bg->empty())
        bgFgExtractor.init(*bg);
    if (!mask.empty())
        bgFgExtractor.setMask(mask);

    if (crops) {
        if (std::any_of(crops->begin(),crops->end(),[](int const crop){return crop > 0;})) {
            bgFgExtractor.setCrop(cv::Rect{(*crops)[0], (*crops)[1], imSize.width - (*crops)[2] - (*crops)[0], imSize.height - (*crops)[3] - (*crops)[1]});
            std::cout << "[" << __func__ << "] " << "set crop to " << bgFgExtractor.getCrop() << std::endl;
        }
    }
//        bgFgExtractor.setCrop(cv::Rect{350,0,imSize.width-120-350,imSize.height});
    std::vector<float> radii;
    std::map<FrameId, CirclesByAreaT> circles;
    std::ofstream fCircles("circles.csv");
    float maxArea = 0.f;
    fCircles << "frameId,x,y" << std::endl;
    for (FrameId frameId = frameIds.front(); frameId != frameIds.back(); step(frameId)) {
        auto const currCircles = bgFgExtractor.update(rgbs.at(frameId), frameId, debug, nCuboids, minBlobArea, learningRate);
        if (!currCircles.size())
            continue;
        circles.emplace(frameId, currCircles);
        for (auto& areaCircle : currCircles) {
            radii.push_back(areaCircle.second.getRadius());
            fCircles << frameId << "," << areaCircle.second.getCentroid().x << "," << areaCircle.second.getCentroid().y << std::endl;
            if (areaCircle.first > maxArea)
                maxArea = areaCircle.first;
        }
    }
    fCircles.close();
    double const timeSec = (cv::getTickCount() - startTime) / cv::getTickFrequency();
    std::cout << "[bgfg] " << timeSec << " sec" << std::endl;

    float diameter = 0.;
    {
        float medianCircleRadius2 = 0.,
              maxCircleRadius2    = 0.;
        std::nth_element(radii.begin(), radii.begin() + radii.size() / 2, radii.end());
        std::cout << "The median is " << radii[radii.size() / 2] << '\n';
        medianCircleRadius2 = radii[radii.size() / 2] * 2.;
        maxCircleRadius2    = *std::max_element(radii.begin(), radii.end()) * 2;
        std::cout << "[" << __func__ << "] " << "medianCircleDiameter:" << medianCircleRadius2 << std::endl;
        std::cout << "[" << __func__ << "] " << "maxCircleDiameter" << maxCircleRadius2 << std::endl;
        diameter = maxCircleRadius2 < medianCircleRadius2 * 1.5 ? maxCircleRadius2
                                                                : medianCircleRadius2;
    }

    if (halfSpreadPos > 0.) {
        for (auto& frameIdCircle : circles) {
            for (auto& areaCircle : frameIdCircle.second) {
                areaCircle.second.getCentroid().x += static_cast<float>(diameter) * (-halfSpreadPos + randf(2. * halfSpreadPos));
                areaCircle.second.getCentroid().y += static_cast<float>(diameter) * (-halfSpreadPos + randf(2. * halfSpreadPos));
            }
        }
    }

//        Scalar const inlierThresh = std::max(static_cast<float>(cv::norm(imageCentroid)) * 0.1f, diameter/inlierDiv); // /2.5f for balls // 3.5 for duckElephant3 // 4.5 for doddle9
    Scalar const inlierThresh = diameter/inlierDiv; // /2.5f for balls // 3.5 for duckElephant3 // 4.5 for doddle9
    std::cout << "[" << __func__ << "] " << "inlier thresh: " << inlierThresh << std::endl;

    EnergyPlotter ep;
    double                  minScore = std::numeric_limits<double>::max();
    BundleWithPhysicsResult best;
    Parabola2ds             bestParabolas;
//        FrameId const quarterTime = frameIds.front() + (static_cast<float>(frameIds.back()) - static_cast<float>(frameIds.front())) / 6.f;
//        std::cout << "[" << __func__ << "] " << "will run between " << 2*quarterTime << " and " << quarterTime * 4 << std::endl;
//#       pragma omp parallel for shared(minScore,best,bestParabolas,ep)
//        for (FrameId midTime = std::max(frameIds.front(),frameIds.at(1)-lookAround); midTime < std::min(frameIds.at(1)+1+lookAround,frameIds.back()); ++midTime) {
    FrameId const midTime = frameIds.at(1);
    {
        Scalar const collTime = midTime + 0.5; //frameIds.at(1) + 0.5;
        int const nPoints = std::count_if(std::begin(circles), std::end(circles), [](decltype(*circles.begin()) const &frameIdCircles) {
            return std::distance(frameIdCircles.second.begin(), frameIdCircles.second.end());
        });
        std::cout << "[" << __func__ << "] " << "midTime: " << midTime << std::endl;
        for (int i= 0; i < 25; ++i) {
            bool keepTrying = true;
            while (keepTrying) {
                cv::Mat img = rgbs.at(frameIds.at(1)).clone();
                cv::Mat debugImg = rgbs.rbegin()->second.clone();
                std::cout << "[" << __func__ << "] " << "keep trying..." << i << std::endl;
                //int nPoints = 0;
                Parabola2ds parabolas;
                for (auto const &frameIdCircles : circles) {
                    FrameId const frameId = frameIdCircles.first;
                    if (std::abs(static_cast<Scalar>(frameId) - collTime) < ignoreAroundCollTime)
                        continue;
                    if (static_cast<int>(frameIdCircles.second.size()) < nCuboids) {
                        continue;
                    }

                    // weight circles by their distance from centroid
                    std::multimap<float, Circle> midWeighted;
                    for (auto const &areaCircle : frameIdCircles.second) {
                        //++nPoints;
                        auto const &cvPnt = areaCircle.second.getCentroid();
//                            float      chance = std::max(0.f, static_cast<float>(cv::norm(imageCentroid - cvPnt) / cv::norm(imageCentroid)) - 0.2f);
                        //float chance = randf();
                        float chance = areaCircle.first / maxArea;
                        midWeighted.insert(std::make_pair(chance, areaCircle.second));
                        char name[255];
                        sprintf(name,"%2.2f", chance);
                        cv::putText(img,name,cvPnt,1,1,cv::Scalar(0.8,0.2,0.8),2);
                    }
                    // insert at most two circles from this frame
                    int id = 0;
                    for (auto iter = midWeighted.rbegin(); iter != midWeighted.rend() && id < nCuboids; ++iter ) {
                        //std::cout << "[" << __func__ << "] " << "skipPoss:" << iter->first << "\n";
//                        if (randf() > iter->first)
//                            continue;
                        Circle const& circle   = iter->second;
                        auto const    &cvPnt   = circle.getCentroid();
                        FrameId const frameId  = frameIdCircles.first;
                        PartId        partId   = frameId < collTime ? 0 : 1;
//                        #warning THREE HACK
                        //CuboidId      cuboidId = cvPnt.x > 650 ? 1 : 0; // first
//                        CuboidId      cuboidId = cvPnt.x > 904 ? 1 : 0; // first
                        CuboidId      cuboidId = nCuboids > 1 ? (randf() > (static_cast<float>(cvPnt.x) / (imageCentroid.x * 2.f)) ? 0
                                                                                                                                   : 1)
                                                              : 0;
                        Parabola2d& parabola2d = parabolas[{cuboidId, partId}];
                        if (parabola2d.hasCircle(frameId))
                            continue;
                        float chance = (static_cast<float>(cvPnt.x) / (imageCentroid.x * 2.f));
                        char text[255];
                        sprintf(text,"%2.1f", chance);
                        cv::putText(debugImg, text, cvPnt, 1, 2, cuboidId ? cv::Scalar(120.,120.,120.) : cv::Scalar(0.,0.,0.));
#if 1
                        auto const otherIt = parabolas.find({!cuboidId,partId});
                        if (otherIt != parabolas.end()) {
                            float minDiff = std::numeric_limits<float>::max();
                            if (otherIt->second.hasCircle(frameId)) {
                                auto const diff = cv::norm(circle.getCentroid() - otherIt->second.getCircle(frameId).getCentroid());
                                if (diff < inlierThresh && diff < minDiff) {
                                    minDiff = diff;
                                    std::cout << "[" << __func__ << "] " << "diff: " << diff << ", frameId: " << frameId << std::endl;
                                    continue;
                                }
                            }
//                                int closeEnough = std::count_if(
//                                    std::begin(otherIt->second.getCircles(frameId)),
//                                    std::end(otherIt->second.getCircles(frameId)),
//                                    [&circle,&minDiff,inlierThresh](decltype(*otherIt->second.getCircles(frameId).cbegin()) circle0){
//                                        auto const diff = cv::norm(circle.getCentroid() - circle0.getCentroid());
//                                        if (diff < inlierThresh && diff < minDiff) {
//                                            minDiff = diff;
//                                            return true;
//                                        } else
//                                            return false;
//                                    });
//                                if (closeEnough)
//                                    continue;
                        }
#endif
//                                if (parabola2d.getCircle(frameId).getArea() > circle.getArea())
//                                    continue;
//                            std::cout << "[" << __func__ << "] " << "cuboidId,partId:" << cuboidId << "," << partId << ", frameId: " << frameId << std::endl;
                        //cv::circle(img,circle.getCentroid(),5,cv::Scalar(cuboidId * 255., partId*255., cuboidId == 0 ? 255. : 0.),2);
                        parabola2d.addCircle(frameId, circle);
                        ++id;
                    }
                } //...for circles
                if (debug) {
                    cv::imshow("debug", debugImg);
//                    cv::imshow("img",img);
                    cv::waitKey(10);
                }

                if (static_cast<int>(parabolas.size()) < nParabolas) {
                    std::cout << "continue0, got " << parabolas.size() << " parabolas <" << nParabolas << std::endl;
                    continue;
                }

                std::cout << "[" << __func__ << "] " << "fit1:" << std::endl;
                double cost = std::numeric_limits<double>::max();
                BundleWithPhysicsResult init = fitParabola(parabolas, frameIds, mapper, weights, cuboids, rgbs, /* show: */ 0, &cost, collTime, flatLimit, saveRawCentroids);
                if (parabolasSimilar(init,nParabolas)) {
                    std::cout << "[" << __func__ << "] " << "parabolas similar..." << std::endl;
                    continue;
                }

                // refit
                Parabola2ds parabolas2;
                auto score = assignCirclesToTracks(circles, *init.tracks3d, inlierThresh, collTime, mapper, &parabolas2, ignoreAroundCollTime, nCuboids);
                if (static_cast<int>(parabolas2.size()) < nParabolas) {
                    std::cout << "continue1, got " << parabolas2.size() << " parabolas <" << nParabolas << std::endl;
                    continue;
                }
                std::cout << "[" << __func__ << "] " << "fit2:" << std::endl;
                init = fitParabola(parabolas2, frameIds, mapper, weights, cuboids, rgbs, /* show: */ 0, &cost, collTime, flatLimit, saveRawCentroids);
                if (parabolasSimilar(init,nParabolas)) {
                    std::cout << "[" << __func__ << "] " << "parabolas similar..." << std::endl;
                    continue;
                }

                // final cost
                score = assignCirclesToTracks(circles, *init.tracks3d, inlierThresh, collTime, mapper, &parabolas2, ignoreAroundCollTime, nCuboids);
                if (static_cast<int>(parabolas2.size()) < nParabolas) {
                    std::cout << "continue2, got " << parabolas2.size() << " parabolas <" << nParabolas << std::endl;
                    continue;
                }

                score.first /= static_cast<Scalar>(cv::norm(imageCentroid)) * 0.1;
                std::cout << "[" << __func__ << "] " << "avgScore: " << score.first << std::endl;
                float explanationRatio = std::max(0.05, 1. - score.second / static_cast<Scalar>(nPoints));
//                    float explanationRatio = std::max(0.05, 1. - score.second / static_cast<Scalar>(circles.size()));
//                    if (explanationRatio > 0.3)
//                        continue;
                const int64 start = cv::getTickCount();
                float similarity = checkPatchSimilarity(parabolas2, rgbs /*,bbox*/);
                const double timeSec = (cv::getTickCount() - start) / cv::getTickFrequency();
                std::cout << "[sift] " << timeSec << " sec" << std::endl;

                std::cout << "[" << __func__ << "] " << "similarity: " << similarity/100.f<< std::endl;
                float sumScore = score.first * 0.2 + score.first * 0.8 * explanationRatio + similarity / 100.f;

                score.first = sumScore;
//            Scalar distWeight = (Eigen::Map<const Eigen::Matrix<Parabola::Scalar,3,1>>{init.parabolas.at(0).at(0).getTranslation().data()}
//                            - Eigen::Map<const Eigen::Matrix<Parabola::Scalar,3,1>>{init.parabolas.at(1).at(0).getTranslation().data()}).norm();
//            std::cout << "[" << __func__ << "] " << "distWeight: " << distWeight << std::endl;
//            score.first *= distWeight;
                std::cout << "[" << __func__ << "] " << "cost: " << cost << ", score: " << score.first << " explaining " << score.second << "/" << nPoints << " (inlierThresh: " << inlierThresh << ")"
                          << std::endl;
#                   pragma omp critical (OMP_CRIT_MINSCORE)
                {
                    if (score.first < minScore && !parabolasSimilar(init,nParabolas)/*&&
                    std::abs(static_cast<float>(static_cast<int>(init.collTimes.at(0))) - init.collTimes.at(0)) > kSmallDiff*/) {
                        minScore      = score.first;
                        best          = std::move(init);
                        bestParabolas = std::move(parabolas2);
                        std::cout << "[" << __func__ << "] " << "stored" << ", stored: " << best.collTimes.at(0) << ", collTime: " << collTime << std::endl;
                    } else {
                        std::cout << "[" << __func__ << "] " << "skipping score " << score.first << ", similar: " << (parabolasSimilar(init,nParabolas) ? "YES" : "NO") << std::endl;
                    }
                }
                keepTrying = false;
            } //...while keepTrying
        } //...for trials
    } //...for collTimes
    std::cout << "[" << __func__ << "] " << "best score: " << minScore << ", with time: " << best.collTimes.at(0)
              << std::endl;
    if (false && debug)
        ep.plot();

    if (parabolasSimilar(best,nParabolas))
        std::cerr << "[" << __func__ << "] " << "best similar" << std::endl;
    BundleWithPhysicsResult output = fitParabola(bestParabolas, frameIds, mapper, weights, cuboids, rgbs,
        /* show: */ debug ? 3 : 0, /* [out] cost: */ nullptr, /* [in] collTime: */ best.collTimes.at(0), flatLimit,
                                                 saveRawCentroids);
    if (parabolasSimilar(output,nParabolas)) {
        std::cerr << "[" << __func__ << "] " << "parabolas similar" << std::endl;
        return best;
    }
    return output;
} //...ransacParabolas()
} //...ns bundle_physics
} //...ns tracking
