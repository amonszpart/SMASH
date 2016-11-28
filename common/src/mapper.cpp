#include "tracking/common/impl/mapper.hpp"
#include "tracking/common/trackPoint2d.h"
#include "tracking/common/trackPoint3d.h"
#include "tracking/common/track.h"
#include "soup/pointCloudEigen.h"

#include "opencv2/imgproc/imgproc.hpp"

namespace tracking {
Mapper::Matrix3 Mapper::getIntrinsics(const INTRINSICS scene) {
    if (scene == TRUCKVIDEO0) {
        Matrix3 intrinsics(getIntrinsics(PRIMESENSE));
        intrinsics(0, 2) += 4.5 + 7.26068 - 9 + 2;
        intrinsics(1, 2) += 11.0 - 10.4063;
        return intrinsics;
    }
    else if (scene == PRIMESENSE) {
        return (Matrix3() << 583.2829786373293, 0.0, 320.0, 0.0, 579.4112549695428, 240.0, 0.0, 0.0, 1.0).finished();
    }
    else if (scene == KINECT_ARON) {
        return (Matrix3() << 583.2829786373293, 0.0, 320.0, 0.0, 579.4112549695428, 240.0, 0.0, 0.0, 1.0).finished();
    }
    else if (scene == DSLR_NILOY) {
//            From exiftools
//            Related Image Width             : 1280
//            Related Image Height            : 720
//            Focal Plane X Resolution        : 177.9755284
//            Focal Plane Y Resolution        : 200.3338898
//            Sensor size                     : 22.5mm x 15mm    from http://www.dpreview.com/products/canon/slrs/canon_eos70d
//
//            Focal length in pixels = (image width in pixels) * (focal length in mm) / (CCD width in mm)
//            fx                              : 1280 * 19mm / 22.5mm = 1080.89 px
//            fy                              :  720 * 19mm / 15.0mm =  912.00 px
        return (Matrix3() << 1080.89, 0.0, 640.0, 0.0, 912., 360.0, 0.0, 0.0, 1.0).finished();
    }
    else if (scene == DSLR_NILOY2) {
        //return (Matrix3() << 980.89, 0.0, 640.0, 0.0, 952., 360.0, 0.0, 0.0, 1.0).finished();
        return (Matrix3() << 960.89, 0.0, 640.0, 0.0, 952., 360.0, 0.0, 0.0, 1.0).finished();
        //return (Matrix3() << 504.125761798782, 0.0, 317.579938053262, 0.0, 505.640834031115, 168.803459758257, 0.0, 0.0, 1.0).finished(); // from matlab
    }
    else if (scene == BLENSOR) {
        // cx: 60.000000, cy: 60.000000, fx: 131.250000, fy: 131.250000
        std::cerr << "[" << __func__ << "]: " << "warning, assumed 640x640 image" << std::endl;
        return (Matrix3() << 700, 0.0, 320.0, 0.0, 700, 320.0, 0.0, 0.0, 1.0).finished();
    }
    else if (scene == IPHONE6) {
        // f= 4.15mm, sensor: 4.8mm ==>
        // 31mm
        return (Matrix3() << 1086, 0.0, 360.0, 0.0, 1086, 640.0, 0.0, 0.0, 1.0).finished();
    }
    else if (scene == IPHONE6HALF) {
        Matrix3 tmp;
        tmp << getIntrinsics(IPHONE6);
        tmp(0, 2) /= 2.;
        tmp(1, 2) /= 2.;
        return tmp;
    }
    else if (scene == IPHONE6HALF_LYING) {
        Matrix3 tmp;
        tmp << getIntrinsics(IPHONE6);
        std::swap(tmp(0, 0), tmp(1, 1));
        std::swap(tmp(0, 2), tmp(1, 2));
        tmp(0, 2) /= 2.;
        tmp(1, 2) /= 2.;
        return tmp;
    }
    else if (scene == S6_LYING) {
        // sensor size: 5.5mm x 4.1mm
        // focal length: 4.3mm (28mm effective)
        // Focal length in pixels = (image width in pixels) * (focal length in mm) / (CCD width in mm)
        // fx_lying = 1280 * 4.3mm / 5.5mm = 1000.727272727 (or 1280 * 4.3mm / 4.1mm = 1342.43902439)
        // fy_lying = 720 * 4.3mm / 4.1mm = 755.12195122 (or 720 * 4.3mm / 5.5mm =  562.909090909)
        Matrix3 tmp;
        tmp << getIntrinsics(IPHONE6);
        std::swap(tmp(0, 0), tmp(1, 1));
        std::swap(tmp(0, 2), tmp(1, 2));
        return tmp;
    }
    else if (scene == S6_BLENDER) { return (Matrix3() << 720.4189, 0.0000, 640.0000,
            0.0000, 720.0000, 360.0000, 0., 0., 1.).finished();

    } else if (scene == IPHONE6_BLENDER) {
        //  Calibrated using matlab:
        //  1186.5    0         630.1
        //  0       1186.6      358.9
        //  0         0          1.0
        return (Matrix3() << 1186.5, 0.0000, 630.1,
            0.0000, 1186.6, 358.9000,
            0.0000, 0.0000, 1.0000).finished();

//            return (Matrix3() << 752.4079,   0.0000, 640.0000,
//                          0.0000, 597.3646, 360.0000,
//                          0.0000,   0.0000,   1.0000).finished();
//            return (Matrix3() << 720.3981,   0.0000, 640.0000,
//                                 0.0000, 660.0000, 360.0000,
//                                 0.0000,   0.0000,   1.0000).finished();

    } else if (scene == DUYGU0) {
        return (Matrix3() << 2358.98, 0., 1281.16, 0., 2358.98, 1021.95, 0., 0., 1.).finished();
    } else
        throw new Mapper_SceneUnknownException("Unknown intrinsics matrix id in Mapper::getIntrinsics()");
}

/**
* \param[in] img Rgb image.
* \param[in] dep Depth image.
* \param[in] normals Optional normals calculated by cv::rgbd::RgbdNormals, and output in CV_32FC1 format.
* \return A shared pointer to the colored cloud.
*/
auto Mapper::depthToCloud(cv::Mat const& img, cv::Mat_<float> const& dep, cv::Mat* normals) const -> CloudPtrT {
    size_t    pId(0);
    CloudPtrT cloud(new CloudT(img.rows * img.cols, 3));
    std::cout << "cloud.size() " << cloud->getNPoints() << std::endl;
    for (int y = 0; y != img.rows; ++y)
        for (int x = 0; x != img.cols; ++x) {
            Scalar depth = Mapper::getDepthValue(dep, Eigen::Vector2i(x, y));
            if (depth < FLT_EPSILON)
                continue;
            cv::Vec3b color = img.at<cv::Vec3b>(y, x);

            cloud->getPoint(pId) = to3D(Eigen::Vector2f(x, y), depth);
            cloud->getColor(pId) = Vector3{ color[0]/255.f, color[1]/255.f, color[2]/255.f };
            if (normals) {
                cv::Vec3f const normal = normals->at<cv::Vec3f>(y, x);
                cloud->getNormal(pId) << normal[0], normal[1], normal[2];
            }
            ++pId;
        }
    if (!normals)
        cloud->clearNormals();
    std::cout << "cloud.size: " << cloud->getNPoints() << std::endl;
    cloud->conservativeResize(pId);
    std::cout << "cloud.size: " << cloud->getNPoints() << std::endl;
    return cloud;
}

//}
//bool operator<(cv::Point_<int> const& lhs, cv::Point_<int> const& rhs) {
//    return lhs.x == rhs.x ? lhs.y < rhs.y
//                          : lhs.x < rhs.x;
//
//}
//namespace tracking {

cv::Mat_<float>
Mapper::cloudToDepth(CloudConstPtrT const& cloud, unsigned const width, unsigned const height,
                     IndexMapT* indices) const {
    return cloudToDepth(cloud, {width, height}, indices);
}

cv::Mat_<float>
Mapper::cloudToDepth(CloudConstPtrT const& cloud, Eigen::Vector2i const& dims, IndexMapT* indices) const {
    if (indices)
        indices->clear();
    cv::Mat_<float> dep {cv::Mat_<float>::zeros(dims(1), dims(0))};
    for (size_t row = 0; row != cloud->getNPoints(); ++row) {
        auto const p2d = to2D(cloud->getPoint(row));
        cv::Point2i coords {static_cast<int>(std::round(p2d.coeff(0))),
                            static_cast<int>(std::round(p2d.coeff(1))) };
        if (coords.x < dep.cols && coords.y < dep.rows && coords.x >=0 && coords.y >= 0) {
            auto const depthVal = cloud->getPoint(row)(2);
            auto &dest = dep.at<float>(coords.y, coords.x);
            if (dest < std::numeric_limits<Scalar>::epsilon() || depthVal < dest) {
                dest = depthVal;
                if (indices)
                    (*indices)[coords] = row;
            }
        }
    } //...for all points
    return dep;
} //...cloudToDepth

cv::Mat Mapper::cloudToRgb(CloudConstPtrT const & cloud, Eigen::Vector2i const& dims) const {
    typedef typename CloudConstPtrT::element_type::Scalar         Scalar;
    typedef typename Soup::geometry::HelperTypes<Scalar>::Vector3 Vector3;
    cv::Mat rgb {cv::Mat::zeros(dims(1), dims(0), CV_8UC3)};
    for (size_t row = 0; row != cloud->getNPoints(); ++row) {
        Vector3 mapped = to2D(cloud->getPoint(row), dims(1) );
        if (mapped.coeff(1) < rgb.rows && mapped.coeff(0) < rgb.cols &&
            mapped.coeff(1) >= 0 && mapped.coeff(0) >= 0) {
                rgb.at<cv::Vec3b>(mapped.coeff(1), mapped.coeff(0)) = cv::Vec3b{
                    static_cast<uchar>(cloud->getColor(row)(2) * 255.f),
                    static_cast<uchar>(cloud->getColor(row)(1) * 255.f),
                    static_cast<uchar>(cloud->getColor(row)(0) * 255.f)};
        }
    }
    return rgb;
} //...cloudToDepth

auto Mapper::to2D(Vector3 const& pnt) const -> Vector2 {
//    return _intrinsics * (Vector3() << pnt.template head<2>() / pnt(2), 1.).finished();
    return (_intrinsics * (pnt / pnt(2))).head<2>();
}

Mapper::Vector3 Mapper::to2D(Vector3 const& pnt, int const height, bool update) const {
    Vector3 tmp = _intrinsics * (Vector3() << pnt.template head<2>() / pnt(2), 1.).finished();
    if ( _invertY )
        tmp(1) = height - tmp(1) - 1;
    if (update) {
        _min = _min.array().min( tmp.array() );
        _max = _max.array().max( tmp.array() );
    }
    return tmp;
}

template float           Mapper::getDepthValue(const cv::Mat& depth, const TrackPoint2D& pnt2 );
template float           Mapper::getDepthValue(const cv::Mat& depth, const TrackPoint2D::Base& pnt2 );
template Mapper::Vector3 Mapper::to3D         (const TrackPoint2D::MyBase& pnt2, const Scalar depth, const int height) const;
template Mapper::Vector3 Mapper::to3D         (const TrackPoint2D        & pnt2, const Scalar depth, const int height) const;
template Mapper::Vector3 Mapper::to3D         (const TrackPoint2D::MyBase& pnt2, const Scalar depth) const;
template Mapper::Vector3 Mapper::to3D         (const TrackPoint2D        & pnt2, const Scalar depth) const;

} //...ns tracking
