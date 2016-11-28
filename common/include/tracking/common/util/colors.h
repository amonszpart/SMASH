#ifndef AM_COLORS_H
#define AM_COLORS_H

#include "tracking/common/eigen.h"
#include "opencv2/core/core.hpp"
#include <vector>

namespace colors
{
    template <typename Scalar>
    inline ::cv::Point3f
    hsv2rgb( ::cv::Point3_<Scalar> const& in );

    /** \brief
     *  \tparam MatrixDervied Concept: Eigen::Matrix<Scalar,3,1>
     */
    template <class MatrixDerived>
    inline MatrixDerived hsv2rgbEigen( MatrixDerived const& in );

    template <typename Scalar>
    inline void rgb2hsv( Scalar r, Scalar g, Scalar b, Scalar &h, Scalar &s, Scalar &v);

    // generates n different colours
    // assumes hue [0, 360), saturation [0, 100), lightness [0, 100)
    template <typename _PointT, typename _Scalar>
    inline std::vector< _PointT >
    nColoursCv(int n, _Scalar scale, bool random_shuffle, float min_value = 50.f, float min_saturation = 90.f );

    template <typename Scalar>
    inline std::vector< ::Eigen::Vector3f >
    nColoursEigen( int n, Scalar scale, bool random_shuffle, float min_value = 50.f, float min_saturation = 90.f );

    inline std::vector< ::Eigen::Vector3f >
    paletteMediumColoursEigen( int min_count = 0, bool repeat = false );

    inline std::vector< cv::Scalar >
    paletteMediumColoursCv( int min_count = 0, bool repeat = false );

    inline std::vector< ::Eigen::Vector3f >
    paletteMediumColoursEigen2( int min_count = 0, float scale = 1., bool random_shuffle = false );

} //...colors

#include "tracking/common/util/impl/colors.hpp"

#endif // AM_COLORS_H
