//
// Created by bontius on 13/06/16.
//

#include "tracking/common/io/cloudIo.h"
#include "tracking/common/util/impl/parse.hpp"
#include "tracking/common/clouds/typedefsCloud.h"
#include "tracking/common/clouds/cloudFactory.h"
#include "soup/pointCloudEigen.h"
#include <iostream>
#include <opencv2/viz.hpp>

namespace tracking {

std::pair<CloudCPtrT, bool> readPly(std::string const& path, Eigen::Matrix4d const& depth2Color) {
    using _Scalar = typename CloudCPtrT::element_type::Scalar;
    typedef Eigen::Map<Eigen::Matrix<uchar, 3, 1> const> Vec3bConstMap;
    typedef Eigen::Map<Eigen::Matrix<float, 3, 1> const> Vec3fConstMap;

    cv::viz::Mesh mesh = cv::viz::Mesh::load(path, cv::viz::Mesh::LOAD_PLY);

    if (mesh.cloud.channels() != 3 || (mesh.cloud.rows != 1 && mesh.cloud.cols != 1)) {
        std::cerr << "[" << __func__ << "] " << "expected cloud to be 1 x n x 3 or n x 1 x 3" << std::endl;
        throw new LogicException("Unexpected format", __FILE__, __LINE__);
    }

    CloudFactory factory;
    for (int row = 0; row != mesh.cloud.rows; ++row)
        for (int col = 0; col != mesh.cloud.cols; ++col) {
            cv::Vec3f cvPx = mesh.cloud.at<cv::Vec3f>(row, col);
            Vec3fConstMap px {&cvPx[0]};
            Eigen::Vector3d const p3 = (depth2Color.block<3,3>(0,0) * (px.cast<double>()) + depth2Color.block<3,1>(0,3));
            if (mesh.colors.rows != 0) {
                factory.addPointWithColor(p3.template cast<_Scalar>(),
                                          Vec3bConstMap{reinterpret_cast<uchar*>(mesh.colors.ptr<cv::Vec3b>(row, col))}.cast<float>() /
                                          255.f);
            }
            else
                factory.addPoint(p3.template cast<_Scalar>());
        }
    if (!factory.size())
        return {nullptr, false};
    else
        return {std::move(factory.create()),true};
} //...loadPly()

int readPlys(ConstCloudsT& clouds, std::string cloudPattern, FrameIdsT const& depthFrameIds, Eigen::Matrix4d const& depth2Color) {
    clouds.clear();
    char cloudPath[2048];
    for (FrameId const frameId : depthFrameIds) {
        sprintf(cloudPath, cloudPattern.c_str(), frameId);

        bool success(false);
        CloudCPtrT cloud;
        std::tie(cloud,success) = readPly(cloudPath, depth2Color);
        if (success)
            clouds.insert(std::make_pair(frameId,cloud));
        else {
            std::cerr << "[" << __func__ << "] " << "could not read " << cloudPath << std::endl;
            return EXIT_FAILURE;
        }
    } //...for frames

    return EXIT_SUCCESS;
} //...readImages()

int readPlys(ConstCloudsT& clouds, int argc, const char** argv, FrameIdsT const& depthFrameIds,
             std::string const& defaultCloudPattern) {
    std::string cloudPattern(defaultCloudPattern);
    if (!console::parse_arg(argc, argv, "--cloud-pattern", cloudPattern) && defaultCloudPattern.empty()) {
        std::cerr << "[" << __func__ << "]: " << "you should specify the input cloud pattern by --cloud-pattern"
                  << std::endl;
        return EXIT_FAILURE;
    }

    Eigen::Matrix4d depth2Color;
    std::string depth2ColorPath("");
    if (console::parse_arg(argc,argv,"--depth2color", depth2ColorPath)) {
        std::cerr << "[" << __func__ << "] " << "reading depth2color matrix is todo" << std::endl;
        throw new std::runtime_error("reading depth2color matrix is todo");
    } else {
        std::cerr << "[" << __func__ << "] " << "using default depth2color matrix (Duygu's structure sensor)"
                  << std::endl;
        depth2Color << 0.999985, 0.00237042, -0.00503879, 0.0350367,
            -0.00232588, 0.999958, 0.00882748, -0.000989049,
            0.0050595, -0.00881562, 0.999948, 0.0223303,
            0., 0., 0., 1.;
    }
    return readPlys(clouds, cloudPattern, depthFrameIds, depth2Color);
} //...readPlys()

} //...ns tracking