//
// Created by bontius on 30/04/16.
//

#ifndef TRACKVIDEO_PHYS_BGFGEXTRACTOR_H
#define TRACKVIDEO_PHYS_BGFGEXTRACTOR_H

#include "tracking/phys/initialize/circle.h"
#include <cv.hpp>
#include <opencv2/core/cuda.hpp>
#include <tracking/common/typedefs.h>
#include <map>

namespace tracking {
  namespace bundle_physics {

    typedef std::vector<ContourT> ContoursT;
    typedef std::map<float, ContourT> SortedContoursT;

//    class Circle {
//        public:
//
//            Circle(cv::Point2f const &centroid, float radius, float area, ContourT const& contour)
//                : centroid(centroid), _radius(radius), _area(area), _contour(contour) {}
//
//            inline cv::Point2f const &getCentroid() const { return centroid; }
//            inline cv::Point2f       &getCentroid()       { return centroid; }
//            float getRadius() const { return _radius; }
//            float getArea() const { return _area; }
//            ContourT const& getContour() const { return _contour;}
//        protected:
//            cv::Point2f centroid;
//            float _radius;
//            float _area;
//            ContourT _contour;
//    }; //...class Circle
//    typedef std::map<float, Circle> CirclesByAreaT;

    class BgFgExtractor {
        public:
                           BgFgExtractor(double const varThreshold = 64, int const erodeIterations = 2, int const closeIterations = 6, int const morphSize = 3, bool const detectShadows = false);
            void           init(cv::Mat const &frame);
            CirclesByAreaT update(cv::Mat const &frame, FrameId const frameId, bool const debug, int const minObjects, int minArea, double learningRate);
            cv::Mat        fillContours(cv::Mat mask);

            void setCrop(cv::Rect const &crop) { _hasCrop = true; _crop = crop; }
            cv::Rect const& getCrop() const { return _crop; }
            void setMask(cv::Mat mask) { cv::bitwise_not(mask,_mask); }
            cv::Mat maskFrame(cv::Mat const& frame);

        protected:
            cv::Ptr<cv::BackgroundSubtractor>  _mog2;
            double                             _mog2_varThreshold;
            cv::Mat                            _smallElement,
                                               _largeElement;
            cv::cuda::GpuMat                   _d_frame, _d_fgmask;
            bool                               _inited;
            bool                               _hasCrop;
            cv::Rect                           _crop;
            int                                _erodeIterations;
            int                                _closeIterations;
            int                                _morphSize;
            int                                _nFrames;
            cv::Mat                            _mask;
    };
  }
}
#endif //TRACKVIDEO_PHYS_BGFGEXTRACTOR_H
