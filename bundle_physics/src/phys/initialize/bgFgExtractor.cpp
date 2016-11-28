//
// Created by bontius on 30/04/16.
//

#define PRE_MASK 1

#include "tracking/phys/initialize/bgFgExtractor.h"
#include <opencv2/cudabgsegm.hpp>
#include <iostream>
#include <map>
#include <tracking/common/typedefs.h>
#include <tracking/common/io/os.h>

struct Compare2f {
    bool operator()(cv::Point_<float> const& a, cv::Point_<float> const& b) const {
        return a.x == b.x ? a.y < b.y : a.x < b.x;
    }
};

namespace tracking {
namespace bundle_physics {


SortedContoursT getSortedContours(cv::Mat const &mask) {
    ContoursT contours;
    std::vector <cv::Vec4i> hierarchy;
    #if 1

//        findContours(mask, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0));
    findContours(mask, contours, hierarchy, cv::RETR_CCOMP, cv::CHAIN_APPROX_TC89_KCOS, cv::Point(0, 0));

    {
        std::map<cv::Point2f, ContourT, Compare2f> heap;
        for (size_t                     i = 0; i < contours.size(); ++i) {
            cv::Point2f center;
            float       radius;
            cv::minEnclosingCircle(contours[i], center, radius);
            heap[center] = contours[i];
        }

        bool found = false;
        do {
            found        = false;
            for (auto it = heap.begin(); it != heap.end() && !found; ++it)
                for (auto it2 = std::next(it); it2 != heap.end() && !found; ++it2) {
                    if (cv::norm(it->first - it2->first) < 100) {
                        auto entry = it->second;
                        entry.insert(entry.end(), it2->second.begin(), it2->second.end());
                        cv::Point2f center = (static_cast<float>(it->second.size()) * it->first +
                                              static_cast<float>(it2->second.size()) * it2->first) /
                                             static_cast<float>(it->second.size() + it2->second.size());
                        auto const  key2   = it2->first;
                        heap.erase(it);
                        heap.erase(key2);
                        heap[center] = entry;
                        found = true;
                    }
                }
        } while (found);
        contours.clear();
        for (auto& entry : heap)
            contours.emplace_back(std::move(entry.second));
    }

//    cv::Mat out;
//    cv::connectedComponents(mask, out, 8);
//    std::cout << "cc: " << cv::sum(out) << "\n";
//    cv::imshow("out", out* 255);
    #else
    // Setup SimpleBlobDetector parameters.
    cv::SimpleBlobDetector::Params params;

// Change thresholds
    params.minThreshold = 10;
    params.maxThreshold = 500;
    params.minDistBetweenBlobs = 50;

// Filter by Area.
    //params.filterByArea = true;
    //params.minArea = 1500;

// Filter by Circularity
    //params.filterByCircularity = true;
    //params.minCircularity = 0.1;

// Filter by Convexity
    //params.filterByConvexity = true;
    //params.minConvexity = 0.87;

// Filter by Inertia
    //params.filterByInertia = true;
    //params.minInertiaRatio = 0.01;


    // Set up detector with params
    cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
    // Detect blobs.
    std::vector<cv::KeyPoint> keypoints;
    detector->detect( mask, keypoints);

// Draw detected blobs as red circles.
// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    cv::Mat im_with_keypoints;
    drawKeypoints( mask, keypoints, im_with_keypoints, cv::Scalar(0,0,255), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    cv::imshow("kp", mask);
    #endif
//        if (contours.size() > 1)
//            std::cerr << "[" << __func__ << "] more, than one contour..." << contours.size() << std::endl;
    SortedContoursT sortedContours;
    for (size_t i = 0; i < contours.size(); ++i) {
        sortedContours.emplace(static_cast<float>(cv::contourArea(contours[i])), contours[i]);
    }
    return sortedContours;
}
#if 0
cv::Mat BgFgExtractor::fillContours(cv::Mat mask) {
        SortedContoursT sortedContours = getSortedContours(mask);

        ContoursT hull(sortedContours.size());
        cv::Mat drawing(cv::Mat::zeros(mask.size(), CV_8UC1));
        int i = 0;
        for (auto iter = sortedContours.rbegin(); iter != sortedContours.rend(); ++iter, ++i) {
            cv::convexHull(cv::Mat(iter->second), hull[i], false);
            cv::drawContours(drawing, hull, (int) i, cv::Scalar(255.), -1, 8, std::vector<cv::Vec4i>(), 0, cv::Point());
            break;
        }
        return drawing;
    }
#endif


/** \param[in] minArea Minimum area of contour to still consider. */
CirclesByAreaT getCentroids(cv::Mat const& mask, double const minArea, double const maxArea) {
    SortedContoursT sortedContours = getSortedContours(mask);
    cv::Point2f     centroid;
    float           radius;
    int             i = 0;
    CirclesByAreaT  circles;
    for (auto iter = sortedContours.rbegin(); iter != sortedContours.rend(); ++iter, ++i) {
        if (minArea > 0. && iter->first < minArea)
            break;
        cv::minEnclosingCircle(iter->second, centroid, radius);
        if (maxArea > 0.&&  iter->first < maxArea)
            circles.emplace(iter->first, Circle{centroid,radius,iter->first,iter->second});
    }
    return circles;
}

cv::Point2i calcCentroid(cv::Mat mask) {
    auto const moments = cv::moments(mask);
    return cv::Point2i(moments.m10 / moments.m00, moments.m01 / moments.m00);
}

BgFgExtractor::BgFgExtractor(double const varThreshold, int const erodeIterations, int const closeIterations, int const morphSize, bool const detectShadows)
    : _mog2(cv::cuda::createBackgroundSubtractorMOG2(100, varThreshold, detectShadows)), _mog2_varThreshold(varThreshold), _inited(false), _hasCrop(false),
      _erodeIterations(erodeIterations), _closeIterations(closeIterations), _morphSize(morphSize), _nFrames(0)
{
    int const morph_elem = cv::MORPH_ELLIPSE;
    _smallElement = cv::getStructuringElement(morph_elem, cv::Size(_morphSize, _morphSize), cv::Point(-1, -1));
    _largeElement = cv::getStructuringElement(morph_elem, cv::Size(2 * _morphSize + 1, 2 * _morphSize + 1), cv::Point(_morphSize, _morphSize));
}

cv::Mat BgFgExtractor::maskFrame(cv::Mat const& frame) {
    cv::Mat tmpFrame(cv::Mat::ones(frame.size(), CV_8UC1) * 255.);
    if (_hasCrop) {
        cv::Mat cropMask(tmpFrame.rows, tmpFrame.cols, CV_8UC1);
        cropMask.setTo(255);
        cv::rectangle(cropMask, _crop.tl(), _crop.br(), cv::Scalar(0.), /* fill: */ -1);
        tmpFrame.setTo(0., cropMask);
    }
    if (!_mask.empty()) {
        tmpFrame.setTo(0., _mask);
    }
    cv::Mat maskedFrame(cv::Mat::zeros(frame.size(),frame.type()));
    frame.copyTo(maskedFrame, tmpFrame);
    return maskedFrame;
}

void BgFgExtractor::init(cv::Mat const &frame) {
    if (_inited) {
        std::cerr << "[" << __func__ << "] " << "Calling init, but already inited...."<< std::endl;
        return;
    }
    _d_frame.create(frame.rows, frame.cols, frame.type());
    _d_frame.upload(frame);
    _mog2->apply(_d_frame, _d_fgmask, 1.);
    _inited = true;
} //...init()

CirclesByAreaT BgFgExtractor::update(cv::Mat const &frame, FrameId const, bool const debug, int const minObjects, int minArea, double learningRate) {
    cv::Mat maskedFrame = maskFrame(frame);
    if (!_inited) {
        init(maskedFrame);
    } else {
        _d_frame.upload(maskedFrame);
        _mog2->apply(_d_frame, _d_fgmask, learningRate);
    }

    cv::Mat fgmask, fgmaskOrig;
    _d_fgmask.download(fgmaskOrig);

    CirclesByAreaT circles;
    if (_nFrames++ < 3)
        return circles;
    int erodeIters = _erodeIterations;
    int closeIters = _closeIterations;
    int tries = 0;
    do {
        fgmask = fgmaskOrig.clone();
//            _d_fgmask.download(fgmask);
#if !PRE_MASK
        if (_hasCrop) {
                cv::Mat cropMask(fgmask.rows, fgmask.cols, CV_8UC1);
                cropMask.setTo(255);
                cv::rectangle(cropMask, _crop.tl(), _crop.br(), cv::Scalar(0.), /* fill: */ -1);
                fgmask.setTo(0., cropMask);
            }
            if (!_mask.empty()) {
                fgmask.setTo(0., _mask);
            }
#endif

        /// Apply the specified morphology operation
        cv::morphologyEx(fgmask, fgmask, cv::MORPH_ERODE, _smallElement, cv::Point2i(-1, -1), erodeIters);
        cv::morphologyEx(fgmask, fgmask, cv::MORPH_CLOSE, _largeElement, cv::Point2i(-1, -1), closeIters);
        //cv::morphologyEx(fgmask, fgmask, cv::MORPH_DILATE, _largeElement, cv::Point2i(-1, -1), closeIters); // synthetic hack

        // Circle tracking
        circles = getCentroids(fgmask, /* minArea: */ minArea, /* maxArea: */ 20000.);
        erodeIters = std::max(0,erodeIters-1);
        closeIters = std::max(0,closeIters-1);
        ++tries;
    } while (static_cast<int>(circles.size()) < minObjects && tries < std::max(_erodeIterations,_closeIterations));
//    cv::imshow("fgmask", fgmask);
//    cv::imshow("fgmaskO", fgmaskOrig);
    static int frameId = 0;
//    io::my_mkdir("bgfgDump");
    char name[512];
//    sprintf(name, "bgfgDump/fgmask_%04d.png", frameId );
//    cv::imwrite(name, fgmask);
//    sprintf(name, "bgfgDump/fgmaskOrig_%04d.png", frameId );
//    cv::imwrite(name, fgmaskOrig);
//    cv::waitKey(10);

    if (debug) {
        cv::Mat tmpMask;
        cv::cvtColor(fgmask,tmpMask,CV_GRAY2BGR);
        for (auto const& areaCircle : circles) {
            cv::circle(tmpMask, areaCircle.second.getCentroid(), areaCircle.second.getRadius(), cv::Scalar(128,200,128));
            char str[255];
//                sprintf(str,"a:%2.2f",areaCircle.first);
//                cv::putText(tmpMask, str, areaCircle.second.getCentroid(), 1, 1.2, cv::Scalar(128,200,128));
            sprintf(str,"r:%2.2f",areaCircle.second.getRadius());
            cv::putText(tmpMask, str, areaCircle.second.getCentroid() + cv::Point2f(0,20), 1, 1.2, cv::Scalar(128,200,128));

        }
        cv::imshow("drawing",tmpMask);
        sprintf(name, "bgfgDump/regions_%04d.png", frameId );
        cv::imwrite(name, tmpMask);
        cv::waitKey(50);
    }
    ++frameId;
    return circles;
#if 0
    cv::Mat     currMask = fgmask.clone();
        cv::Mat     blob     = fillContours(currMask);
        cv::Point2i cp       = calcCentroid(blob);
        cv::rectangle(currMask, cp - cv::Point2i(20, 20), cp + cv::Point2i(20, 20), cv::Scalar(0., 0., 0.));
        blob = fillContours(currMask);
        cv::Point2i cp2 = calcCentroid(blob);

        currMask = fgmask.clone();
        cv::circle(currMask, cp, 3, cv::Scalar(128, 128, 128), 2);
        cv::circle(currMask, cp2, 3, cv::Scalar(128, 128, 128), 2);
        cv::imshow("[BgFgExtractor] drawing", currMask);

        cv::imshow("[BgFgExtractor] foreground mask", fgmask);
        cv::waitKey(50);
        return std::vector<cv::Point2f> {cp, cp2};
#endif
}
}
}
