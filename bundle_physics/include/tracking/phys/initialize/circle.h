//
// Created by bontius on 09/05/16.
//

#ifndef TRACKVIDEO_PHYS_CIRCLE_H
#define TRACKVIDEO_PHYS_CIRCLE_H

#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"

#include <vector>
#include <map>

namespace tracking {
  namespace bundle_physics {
    typedef std::vector<cv::Point> ContourT;

    class Circle {
        public:

            Circle(cv::Point2f const &centroid, float radius, float area, ContourT const& contour)
                : centroid(centroid), _radius(radius), _area(area), _contour(contour) {}
            Circle(Circle const& other) = default;
            Circle(Circle && other) = default;
            Circle& operator=(Circle const& other) = default;
            Circle& operator=(Circle && other) = default;
            virtual ~Circle() = default;

            inline cv::Point2f const &getCentroid() const { return centroid; }
            inline cv::Point2f       &getCentroid()       { return centroid; }
            float getRadius() const { return _radius; }
            float getArea() const { return _area; }
            ContourT const& getContour() const { return _contour;}
            cv::Rect getBoundingBox() const { return cv::boundingRect(cv::Mat(_contour)); }
        protected:
            cv::Point2f centroid;
            float _radius;
            float _area;
            ContourT _contour;
    }; //...class Circle
    typedef std::map<float, Circle> CirclesByAreaT;

  } //...ns bundle_phsyics
} //...ns tracking
#endif //TRACKVIDEO_PHYS_CIRCLE_H
