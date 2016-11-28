#ifndef TRACKING_COMMON_MAPPER_H
#define TRACKING_COMMON_MAPPER_H

#include "tracking/common/eigen.h"
#include "tracking/common/util/exception.h"
#include "tracking/common/clouds/typedefsCloud.h"
#include "soup/cloudTypedefs.h"
#include "soup/geometryTypedefs.h"
#include "opencv2/core.hpp"
#include <iostream>
#include <map>
#include <tracking/common/clouds/typedefsCloud.h>

namespace tracking {

struct CompareCvPoint2i: public std::binary_function<cv::Point2i,cv::Point2i, bool> {
    bool operator()(cv::Point2i const& lhs, cv::Point2i const& rhs) const {
        return lhs.x == rhs.x ? lhs.y < rhs.y
                              : lhs.x < rhs.x;
    }
};
typedef std::map<cv::Point2i, size_t, CompareCvPoint2i> IndexMapT;

/**
 * \brief Class to transform clouds between 2D and 3D.
 * From: http://codeyarns.com/2015/09/08/how-to-compute-intrinsic-camera-matrix-for-a-camera/
 */
class Mapper {
    DEFINE_EXCEPTION(Mapper_SceneUnknown)
    DEFINE_EXCEPTION(Mapper_DepthMapNot32FC1);
public:
    enum INTRINSICS {
        PRIMESENSE,
        TRUCKVIDEO0,
        KINECT_ARON,
        DSLR_NILOY,
        DSLR_NILOY2,
        BLENSOR,
        IPHONE6,
        IPHONE6HALF,
        IPHONE6HALF_LYING,
        S6_LYING,
        S6_BLENDER,
        IPHONE6_BLENDER, //!< Used in physacq (Niloy's calibrated iphone)
        DUYGU0           //!< Duygu's structure sensor (13 June 2016)
    };

    typedef Soup::geometry::Scalar             Scalar;
    typedef typename Eigen::Matrix<Scalar,3,3> Matrix3;
    typedef          Eigen::Matrix<Scalar,2,1> Vector2;
    typedef          Eigen::Matrix<Scalar,3,1> Vector3;

    Mapper()
        : _intrinsics( Matrix3::Zero() )
        , _min(Vector3::Ones() * std::numeric_limits<Vector3::Scalar>::max())
        , _max(Vector3::Zero())
        , _offset( Vector2::Zero() )
        , _invertY(false) {}

    Mapper( const Matrix3& intrinsics )
        : _intrinsics( intrinsics )
        , _min(Vector3::Ones() * std::numeric_limits<Vector3::Scalar>::max())
        , _max(Vector3::Zero())
        , _offset( Vector2::Zero() )
        , _invertY(false) {}
    inline void           setMin( const Vector3& mn ) { _min = mn; }
    inline void           setMax( const Vector3& mx ) { _max = mx; }
    inline const Vector3& min() const { return _min; }
    inline const Vector3& max() const { return _max; }
    Vector3 to2D(const Vector3& pnt, const int height, bool update = false ) const;
    Vector2 to2D(Vector3 const& pnt) const;

    template <typename _Vector2>
    Vector3 to3D( const _Vector2& pnt2, const Scalar depth, const int height ) const;
    template <typename _Vector2>
    Vector3 to3D( const _Vector2& pnt2, const Scalar depth ) const;

    inline void biasValues( std::vector<Vector3>& points ) { for ( auto& point : points ) point -= _min; }
    inline Vector3 getDims() const { return (_max - _min).array().ceil(); }

    static       Matrix3  getIntrinsics( const INTRINSICS scene );
    inline const Matrix3& getIntrinsics() const { return _intrinsics; }
    inline       Matrix3& getIntrinsics()       { return _intrinsics; }

    Mapper( const INTRINSICS intrinsics ) : Mapper( getIntrinsics(intrinsics) ) {}

    /** \brief Converts rgb and depth images to colored pointclouds. */
    CloudPtrT depthToCloud(const cv::Mat& img, const cv::Mat_<float>& depth, cv::Mat* normals = nullptr) const;

    cv::Mat_<float>
    cloudToDepth(CloudConstPtrT const& cloud, const Eigen::Vector2i& dims, IndexMapT* indices = nullptr) const;
    cv::Mat_<float>
    cloudToDepth(CloudConstPtrT const& cloud, unsigned const width, unsigned const height,
                 IndexMapT* indices = nullptr) const;

    cv::Mat cloudToRgb(const CloudConstPtrT& cloud, const Eigen::Vector2i& dims) const;

    template <typename _Vector2>
    static float getDepthValue( const cv::Mat& depth, const _Vector2& pnt2 );
    template <typename _Vector2>
    static float getClosestDepthValue( const cv::Mat& depth, const _Vector2& pnt2, float minValue, int maxRange = 5 );

    inline Scalar getFx() const { return _intrinsics(0,0); }
    inline Scalar getFy() const { return _intrinsics(1,1); }
    inline Scalar getCx() const { return _intrinsics(0,2); }
    inline Scalar getCy() const { return _intrinsics(1,2); }
protected:
    Matrix3         _intrinsics;
    mutable Vector3 _min, _max;
    Vector2         _offset;
    bool            _invertY;
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
}; //...cls Mapper

} //...ns tracking
#endif // TRACKING_COMMON_MAPPER_H
