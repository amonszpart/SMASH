//
// Created by bontius on 11/04/16.
//

#include "tracking/phys/initialize/assignments.h"
#include "tracking/phys/initialize/parabola2d.h"
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/partUtil.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/util/util.h"
#include "tracking/common/io/os.h"
#include "opencv2/highgui.hpp"
#include <list>

namespace tracking {
  namespace bundle_physics {

  void saveBlobs(BundleWithPhysicsResult const& init, GroupedTracks2d const& selectedTracks2d,
                 TracksToCuboidsT const& assignments, RgbsT const& indexedRgbs, FrameIdsT const& frameIds,
                 Mapper const& mapper) {
        Scalar const threshold2d = 100.f;
        io::my_mkdir("patches");
        for (FrameId frameId = frameIds.front(); frameId <= frameIds.back(); step(frameId)) {
            std::map<CuboidId,std::list<cv::Point2i>> pointsInFrame;

            for (auto const& pair : init.tracks3d->getGroups()) {
                if (pair.second.size() > 1) {
                    std::cerr << "[" << __func__ << "] more, than just a parabola in input..." << pair.second.size() << std::endl;
                    throw new std::runtime_error("");
                }

                Track3D const& track3D = init.tracks3d->at(*pair.second.begin());
                if (!track3D.hasPoint(frameId))
                    continue;
                TrackPoint3D const& point3D = track3D.getPoint(frameId);
                auto         const  p2      = mapper.to2D(point3D.getPoint());

                for (auto const& track2d : selectedTracks2d) {
                    TrackId const& trackId = track2d.getTrackId();

                    auto pntIter = track2d.findPoint(frameId);
                    if (pntIter == track2d.end())
                        continue;

                    auto iter = assignments.find(trackId);
                    if (iter == assignments.end())
                        continue;
                    CuboidId cuboidId = iter->second;
                    if ((pntIter->second - p2.head<2>()).norm() < threshold2d)
                        pointsInFrame[cuboidId].push_back(cv::Point2i(pntIter->second(0),pntIter->second(1)));
                } //...for assignments
            } //...for groups

            for (auto const& cuboidIdAndPoints : pointsInFrame) {
                cv::Mat rgb(indexedRgbs.at(frameId).clone());
                cv::Mat mask(cv::Mat::zeros(rgb.size(),CV_8UC1));

                cv::Scalar color((cuboidIdAndPoints.first == 0) * 255., cuboidIdAndPoints.first * 255., 0.);
                for (cv::Point2i const &point2i : cuboidIdAndPoints.second) {
                    cv::circle(rgb, point2i, 1., color);
                    cv::circle(mask, point2i, 2., cv::Scalar(255., 255., 255.), -1);
                }

                std::vector<std::vector<cv::Point> > contours;
                std::vector<cv::Vec4i>               hierarchy;
                cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0));
                cv::imshow("mask", mask);
                cv::waitKey(10);
                if (contours.size() > 1)
                    std::cerr << "[" << __func__ << "] more, than one contour..." << contours.size() << std::endl;
                std::vector<std::vector<cv::Point> > hull(contours.size());

                cv::Mat drawing(cv::Mat::zeros(rgb.size(), CV_8UC1));
                for (size_t i = 0; i < contours.size(); ++i) {
                    cv::convexHull(cv::Mat(contours[i]), hull[i], false);
                    cv::drawContours(drawing, hull, (int) i, cv::Scalar(255.), -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
                }

                cv::Mat final(cv::Mat::ones(rgb.size(), CV_8UC3));
                final.setTo(cv::Scalar(255.,255.,255.));
                indexedRgbs.at(frameId).copyTo(final, drawing);
                cv::imshow("final", final);
                char name[255];
                sprintf(name,"patches/cuboid%d_frame%03u.png",cuboidIdAndPoints.first,frameId);
                cv::imwrite(name,final);
                cv::waitKey(50);
                cv::imshow("hull", rgb);

                cv::waitKey(100);
            }
        } //...for frames
    } //...saveBlobs()

    cv::Mat fillContours(cv::Mat mask) {
        std::vector<std::vector<cv::Point> > contours;
        std::vector<cv::Vec4i>               hierarchy;
        cv::findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0));
        cv::imshow("fillContours", mask);

        if (contours.size() > 1)
            std::cerr << "[" << __func__ << "] more, than one contour..." << contours.size() << std::endl;
        std::vector<std::vector<cv::Point> > hull(contours.size());

        std::map<Scalar,int> sortedContourIds;
        for (size_t i = 0; i < contours.size(); ++i) {
            sortedContourIds.emplace(cv::contourArea(contours[i]),i);
        }

        cv::Mat drawing(cv::Mat::zeros(mask.size(), CV_8UC1));
        for (auto iter = sortedContourIds.rbegin(); iter != sortedContourIds.rend(); ++iter) {
            int const i = iter->second;
            //for (size_t i = 0; i < contours.size(); ++i) {
            cv::convexHull(cv::Mat(contours[i]), hull[i], false);
            cv::drawContours(drawing, hull, (int) i, cv::Scalar(255.), -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
            break;
        }
        cv::imshow("drawing",drawing);
        cv::waitKey(10);
        return drawing;
    }

    /** \brief Uses input rect size to cut around parabola. */
    void saveBlobs2(Eigen::Vector2f const& roi, BundleWithPhysicsResult const& init, RgbsT const& indexedRgbs,
                    FrameIdsT const& frameIds, Mapper const& mapper, cv::Mat const* bg) {
        using _Scalar = tracking::bundle_physics::Scalar;
        io::my_mkdir("patches");

//        int const morph_size = 3;
//        int const morph_elem = cv::MORPH_ELLIPSE;
//        cv::Mat smallElement = cv::getStructuringElement(morph_elem, cv::Size(3,3), cv::Point(0,0) );
//        cv::Mat largeElement = cv::getStructuringElement(morph_elem, cv::Size(2*morph_size + 1,2*morph_size+1), cv::Point(morph_size,morph_size));
        for (auto const& entry : init.parabolas) {
            CuboidId const cuboidId = entry.first;
            if (!init.cuboids->at(cuboidId).isMassFinite()) {
                continue;
            }

            for (FrameId frameId = frameIds.front(); frameId <= frameIds.back(); step(frameId)) {
                auto const rgbIter = indexedRgbs.find(frameId);
                if (rgbIter == indexedRgbs.end()) {
                    std::cerr << "[" << __func__ << "] " << "could not find rgb with frameId " << frameId << std::endl;
                    continue;
                }
                cv::Mat rgb = rgbIter->second;
                cv::Mat diff;
                if (bg) {
                    cv::absdiff(rgb, *bg, diff);
                    //cv::subtract(rgb, *bg, diff);
                } else {
                    diff = rgb;
                }
                PartId const partId = getPartId(frameId, frameIds);
                std::cout << "[" << __func__ << "] " << "querying " << cuboidId << " and " << partId
                          << "(partId) for frameId " << frameId << std::endl;

                auto const cuboidIter = init.parabolas.find(cuboidId);
                if (cuboidIter == init.parabolas.end()) {
                    std::cerr << "[" << __func__ << "] " << "could not find parabola[cuboidId=" << cuboidId << "]"
                              << std::endl;
                    continue;
                }
                auto const partIter = cuboidIter->second.find(partId);
                if (partIter == cuboidIter->second.end()) {
                    std::cerr << "[" << __func__ << "] " << "could not find parabola[cuboidId=" << cuboidId
                              << "][partId=" << partId << "]" << std::endl;
                    continue;
                }

                auto const centroid = mapper.to2D(
                    partIter->second.getPosition(frameId, init.rotX, init.rotY1, init.a, init.collTimes.at(0)).cast<_Scalar>());
                std::cout << "[" << __func__ << "] " << "centroid: " << centroid.transpose() << std::endl;
                cv::Rect rect(
                    std::min(diff.cols - 1.f, std::max(0.f, centroid(0) - roi(0) / 2)),
                    std::min(diff.rows - 1.f, std::max(0.f, centroid(1) - roi(1) / 2)),
                    roi(0),
                    roi(1));
                if (rect.x + rect.width > diff.cols) {
                    rect.width = diff.cols - rect.x;
                }
                if (rect.y + rect.height > diff.rows) {
                    rect.height = diff.rows - rect.y;
                }
                if (rect.width < 0 || rect.height < 0) {
                    continue;
                }
                std::cout << "[" << __func__ << "] " << "rect: " << rect << std::endl;
                cv::Mat gray;
                cv::cvtColor(diff(rect), gray, CV_RGB2GRAY);
                cv::Mat threshed;
                cv::threshold(gray, threshed, 8, 255., CV_THRESH_BINARY);
                cv::imshow("gray", gray);
                cv::moveWindow("gray", 1970, 50);

//                cv::morphologyEx( gray, gray, cv::MORPH_ERODE, smallElement, cv::Point2i(-1,-1), 1 );
//                cv::morphologyEx( gray, gray, cv::MORPH_CLOSE, largeElement, cv::Point2i(-1,-1), 3 );

                cv::moveWindow("mask", 1970 + gray.cols, 50);
                cv::imshow("mask", threshed);

                threshed = fillContours(threshed);

                cv::Mat masked;
                rgb(rect).copyTo(masked, threshed);
                cv::moveWindow("roi", 1970 + 2 * gray.cols, 50);
                cv::imshow("roi", masked);

                char name[255];
                sprintf(name, "patches/cuboid%d_frame%03u.png", cuboidId, frameId);
                cv::imwrite(name, masked);

                ::io::my_mkdir("canny");
                cv::Mat cannyIn;
                cv::Canny(rgb(rect), cannyIn, 0.5 * 255, 0.9 * 255);
                sprintf(name, "canny/cuboid%d_frame%03u.png", cuboidId, frameId);
                cv::imwrite(name, cannyIn);
                cv::imshow("cannyIn", cannyIn);
                cv::waitKey(50);
            } //...for frames
        } //...for cuboids
    } //...saveBlobs2()
  } //...ns bundle_physics
} //...ns tracking