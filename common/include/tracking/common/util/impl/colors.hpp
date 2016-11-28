#ifndef AM_COLORS_HPP
#define AM_COLORS_HPP

#include "opencv2/core/core.hpp"
#include <vector>

namespace colors {
    template <typename Scalar>
    inline ::cv::Point3f
    hsv2rgb( ::cv::Point3_<Scalar> const& in )
    {
        double      hh, p, q, t, ff;
        long        i;
        ::cv::Point3f out;

        if ( in.y <= 0.f ) // < is bogus, just shuts up warnings
        {
            out.x = in.z;
            out.y = in.z;
            out.z = in.z;
            return out;
        }

        hh = in.x;
        if ( hh >= 360.f ) hh = 0.f;
        hh /= 60.f;
        i = (long)hh;
        ff = hh - i;
        p = in.z * (1.f - in.y);
        q = in.z * (1.f - (in.y * ff));
        t = in.z * (1.f - (in.y * (1.f - ff)));

        switch(i) {
            case 0:
                out.x = in.z;
                out.y = t;
                out.z = p;
                break;
            case 1:
                out.x = q;
                out.y = in.z;
                out.z = p;
                break;
            case 2:
                out.x = p;
                out.y = in.z;
                out.z = t;
                break;

            case 3:
                out.x = p;
                out.y = q;
                out.z = in.z;
                break;
            case 4:
                out.x = t;
                out.y = p;
                out.z = in.z;
                break;

            case 5:
            default:
                out.x = in.z;
                out.y = p;
                out.z = q;
                break;
        }

        return out;
    }

    //static void RGB2HSV(float r, float g, float b,
    //                    float &h, float &s, float &v)
    //{
    //    float K = 0.f;

    //    if (g < b)
    //    {
    //        std::swap(g, b);
    //        K = -1.f;
    //    }

    //    if (r < g)
    //    {
    //        std::swap(r, g);
    //        K = -2.f / 6.f - K;
    //    }

    //    float chroma = r - std::min(g, b);
    //    h = fabs(K + (g - b) / (6.f * chroma + 1e-20f));
    //    s = chroma / (r + 1e-20f);
    //    v = r;
    //}

    /*! \brief
     *  \tparam MatrixDervied Concept: Eigen::Matrix<Scalar,3,1>
     */
    template <class MatrixDerived>
    inline MatrixDerived
    hsv2rgbEigen( MatrixDerived const& in )
    {
        typedef typename MatrixDerived::Scalar Scalar;

        double      hh, p, q, t, ff;
        long        i;
        MatrixDerived out;

        if ( in(1) <= Scalar(0.) ) // < is bogus, just shuts up warnings
        {
            out(0) = in(2);
            out(1) = in(2);
            out(2) = in(2);
            return out;
        }

        hh = in(0);
        if ( hh >= Scalar(360.) ) hh = Scalar(0.);
        hh /= Scalar(60.);
        i = (long)hh;
        ff = hh - i;
        p = in(2) * (1.f - in(1));
        q = in(2) * (1.f - (in(1) * ff));
        t = in(2) * (1.f - (in(1) * (1.f - ff)));

        switch(i) {
            case 0:
                out(0) = in(2);
                out(1) = t;
                out(2) = p;
                break;
            case 1:
                out(0) = q;
                out(1) = in(2);
                out(2) = p;
                break;
            case 2:
                out(0) = p;
                out(1) = in(2);
                out(2) = t;
                break;

            case 3:
                out(0) = p;
                out(1) = q;
                out(2) = in(2);
                break;
            case 4:
                out(0) = t;
                out(1) = p;
                out(2) = in(2);
                break;

            case 5:
            default:
                out(0) = in(2);
                out(1) = p;
                out(2) = q;
                break;
        }

        return out;
    }

    template <typename Scalar> inline
    static void rgb2hsv(Scalar r, Scalar g, Scalar b, Scalar &h, Scalar &s, Scalar &v)
    {
        Scalar K(0.);
        if (g < b)
        {
            std::swap(g, b);
            K = -1.;
        }
        Scalar min_gb(b);
        if (r < g)
        {
            std::swap(r, g);
            K = -2. / 6. - K;
            min_gb = std::min(g, b);
        }
        Scalar chroma = r - min_gb;
        h = fabs(K + (g - b) / (6. * chroma + 1.e-20));
        s = chroma / (r + 1.e-20);
        v = r;
    }

    // generates n different colours
    // assumes hue [0, 360), saturation [0, 100), lightness [0, 100)
    template <typename _PointT, typename _Scalar>
    inline std::vector<_PointT>
    nColoursCv(int n, _Scalar scale, bool random_shuffle, float min_value, float min_saturation)
    {
        std::vector< _PointT > out;
        //srand(time(NULL));

        float step = 360. / n;
        const float saturation_rest = 100.f - min_saturation;
        const float value_rest = 100.f - min_value;
        for ( int i = 0; i < n; ++i )
        {
            cv::Point3f hsv;
            hsv.x = i * step; // hue
            hsv.y = (min_saturation + rand()/(float)RAND_MAX * saturation_rest) / 100.f; // saturation
            hsv.z = (min_value      + rand()/(float)RAND_MAX * value_rest     ) / 100.f; // value

            // convert and store
            cv::Point3f rgb = hsv2rgb(hsv) * scale;
            out.emplace_back( _PointT(rgb.x, rgb.y, rgb.z) );
        }

        if ( random_shuffle )
            std::random_shuffle( out.begin(), out.end() );

        return out;
    }

    template <typename Scalar>
    inline std::vector< ::Eigen::Vector3f >
    nColoursEigen(int n, Scalar scale, bool random_shuffle, float min_value, float min_saturation)
    {
        typedef std::vector< ::cv::Point3f > CvPointsT;
        CvPointsT colours = nColoursCv<::cv::Point3f>( n, scale, random_shuffle, min_value, min_saturation );
        std::vector< ::Eigen::Vector3f > colours_eigen;
        //for ( CvPointsT::value_type const& colour : colours )
        for ( CvPointsT::const_iterator it = colours.begin(); it != colours.end(); ++it )
        {
            //colours_eigen.push_back( (Eigen::Vector3f() << colour.x, colour.y, colour.z).finished() );
            colours_eigen.push_back( (Eigen::Vector3f() << it->x, it->y, it->z).finished() );
        }

        return colours_eigen;
    }

    inline std::vector< ::Eigen::Vector3f >
    paletteMediumColoursEigen(int min_count, bool repeat)
    {
        // don't have template parameters, since we return Vector3f....
        static std::vector< ::Eigen::Vector3f > colours_eigen;
        if ( !colours_eigen.size() )
        {
            colours_eigen.resize( 10 );
            colours_eigen[0] << 241.f, 090.f, 096.f;
            colours_eigen[1] << 122.f, 195.f, 106.f;
            colours_eigen[2] << 090.f, 155.f, 212.f;
            colours_eigen[3] << 250.f, 167.f, 091.f;
            colours_eigen[4] << 158.f, 103.f, 171.f;
            colours_eigen[5] << 206.f, 112.f, 088.f;
            colours_eigen[6] << 215.f, 127.f, 180.f;
            colours_eigen[7] << 241.f, 124.f, 176.f; // added by Aron
            colours_eigen[8] << 182.f, 160.f,  96.f; // added by Aron
            colours_eigen[9] << 136.f, 136.f, 136.f; // added by Aron
            for ( auto &color : colours_eigen )
                color /= 255.f;
        }

        std::vector< ::Eigen::Vector3f > out;
        do out.insert( out.end(), colours_eigen.begin(), colours_eigen.end() );
        while ( repeat && static_cast<int>(out.size()) < min_count );

        return out;
    }

    inline std::vector< cv::Scalar >
    paletteMediumColoursCv(int min_count, bool repeat)
    {
        auto colors = paletteMediumColoursEigen( min_count, repeat );
        std::vector< cv::Scalar > out( colors.size() );
        for (size_t i = 0; i < colors.size(); ++i)
            out[i] = cv::Scalar( colors[i](0) * 255.f, colors[i](1) * 255.f, colors[i](2) * 255.f );
        return out;
    }

    inline std::vector< ::Eigen::Vector3f >
    paletteMediumColoursEigen2(int min_count, float scale, bool random_shuffle)
    {
        // don't have template parameters, since we return Vector3f....
        static std::vector< ::Eigen::Vector3f > colours_eigen;

        if ( !colours_eigen.size() )
        {
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.478431, 0.764706, 0.415686));//, Fruit salad ~
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.352941, 0.607843, 0.831373));//, Curious blue ~
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.980392, 0.654902, 0.356863));//, Rajah ~
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.619608, 0.403922, 0.670588));//, Clairvoyant ~
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.807843, 0.439216, 0.345098));//, Japonica ~
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.945098, 0.352941, 0.376471));//, Valencia ~
            colours_eigen.emplace_back(scale*Eigen::Vector3f(.843137, 0.498039, 0.705882));//, Shocking ~
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.45098, 0.45098, 0.45098));//, Dim gray ~
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.945098, 0.352941, 0.377073));//, scheme Square #0[Valencia]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.486808, 0.352941, 0.945098));//, scheme Square #1[Dark blue]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.36085, 0.945098, 0.352941));//, scheme Square #2[Lime green]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.478701, 0.764706, 0.415686));//, scheme Square #0[Fruit salad]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.764706, 0.65592, 0.415686));//, scheme Square #1[Putty]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.764706, 0.415686, 0.493995));//, scheme Square #2[Charm]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.352941, 0.608355, 0.831373));//, scheme Square #0[Curious blue]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.713493, 0.831373, 0.352941));//, scheme Square #1[Limerick]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.831373, 0.592841, 0.352941));//, scheme Square #2[Porsche]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.980392, 0.654042, 0.356863));//, scheme Square #0[Rajah]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.980392, 0.356863, 0.972055));//, scheme Square #1[Dark magenta]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.356863, 0.741702, 0.980392));//, scheme Square #2[Curious blue]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.619343, 0.403922, 0.670588));//, scheme Square #0[Clairvoyant]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.403922, 0.64891, 0.670588));//, scheme Square #1[Cadet blue]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.654801, 0.670588, 0.403922));//, scheme Square #2[Green smoke]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.807843, 0.438632, 0.345098));//, scheme Square #0[Japonica]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.578529, 0.345098, 0.807843));//, scheme Square #1[Indigo]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.345098, 0.807843, 0.596853));//, scheme Square #2[Eucalyptus]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.843137, 0.498039, 0.706418));//, scheme Square #0[Shocking]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.498039, 0.554971, 0.843137));//, scheme Square #1[Chetwode blue]
            colours_eigen.emplace_back(scale*Eigen::Vector3f(0.672381, 0.843137, 0.498039));//, scheme Square #2[Feijoa]
        }

        std::vector< ::Eigen::Vector3f > out;
        do
        {
            out.insert( out.end(), colours_eigen.begin(), colours_eigen.end() );
        }
        while ( static_cast<int>(out.size()) < min_count );

        if (random_shuffle)
            std::random_shuffle( out.begin(), out.end() );

        return out;
    }

} //...colors

#endif // AM_COLORS_HPP
