#include "tracking/common/io/pgm.h"
#include "opencv2/core/core.hpp"
#include <iostream>
#include <vector>

namespace tracking {
    bool readPgm(std::string const& path, cv::Mat_<float>& cvDepth, float const scale = 1.f) {
        FILE* fp = fopen( path.c_str(), "r" );
        if ( !fp )
            return false;

        std::string header = one_line( fp );
        int firstSpace = header.find(" ");
        int secondSpace = header.find(" ", firstSpace+1 );
        int thirdSpace = header.find(" ", secondSpace+1 );

        std::vector<uint16_t> depth;

        int depthWidth = std::atoi( header.substr( firstSpace+1, secondSpace - firstSpace - 1 ).c_str() );
        //std::cout << "width: " << depthWidth << std::endl;
        int depthHeight = std::atoi( header.substr( secondSpace+1, thirdSpace - secondSpace - 1 ).c_str() );
        //std::cout << "height: " << depthHeight << std::endl;
        depth.resize( depthWidth * depthHeight );
        //std::cout << "header: " << header << std::endl;

        size_t read = fread( depth.data(), sizeof(uint16_t), depth.size(), fp );
        if ( read != depth.size() )
            std::cerr << "[" << __func__ << "] read " << read << " instead of " << depth.size() << std::endl;
        if ( fp )
            fclose( fp );

        //depth = cv::imread( pair.first.string(), CV_LOAD_IMAGE_ANYDEPTH | CV_LOAD_IMAGE_ANYCOLOR );
        //depthRgb = cv::Mat::zeros( depth.rows, depth.cols, CV_8UC3 );
        //std::cout << depth.channels() << std::endl;
        //std::cout << depth.type() << "," << depth.depth() << std::endl;
        cvDepth.create(depthHeight, depthWidth);
        int index = 0;
        for (int y = 0; y != depthHeight; ++y)
            for (int x = 0; x != depthWidth; ++x, ++index)
                cvDepth.at<float>(y, x) = depth.at(index) * scale;

        return true;
    }
} //...tracking
