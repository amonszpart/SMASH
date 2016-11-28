//
// Created by bontius on 11/07/16.
//

#include <tracking/common/io/flowIo.h>

#include <gtest/gtest.h>
#include <random>
#include <opencv2/core.hpp>
#include <cstdio>
#include "tracking/common/io/os.h"

class FlowIOTestFixture : public ::testing::Test {
    public:
        FlowIOTestFixture() : _testFilePath("flowTest.flo") {
            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_real_distribution<> dis(-1,1);
            _flowX.create(480,640,CV_32FC1);
            _flowY.create(480,640,CV_32FC1);
            for (int row = 0; row != _flowX.rows; ++row)
                for (int col = 0; col != _flowX.cols; ++col) {
                    _flowX.at<float>(row,col) = static_cast<float>(dis(gen));
                    _flowY.at<float>(row,col) = static_cast<float>(dis(gen));
                }
        } //...FlowIOTestFixture()

    protected:
        cv::Mat _flowX, _flowY;
        std::string const _testFilePath;
};

TEST_F(FlowIOTestFixture,RandomFlow_FC2) {
    ::io::removeFile(_testFilePath);
    ASSERT_FALSE(io::exists(_testFilePath));
    io::flow::writeFlowFile(_flowX,_flowY,_testFilePath);
    ASSERT_TRUE(io::exists(_testFilePath));

    std::vector<cv::Mat> readFlow;
    cv::Mat_<cv::Vec2f> readFlow2 = io::flow::readFlowFile(_testFilePath);
    cv::split(readFlow2, readFlow);
    ASSERT_EQ(readFlow2.size(), _flowX.size());

    ASSERT_EQ(readFlow[0].size(), _flowX.size());
    cv::Mat diff;
    cv::compare(readFlow[0], _flowX, diff, cv::CMP_NE);
    EXPECT_EQ(0, cv::countNonZero(diff));

    ASSERT_EQ(readFlow[1].size(), _flowY.size());
    cv::compare(readFlow[1], _flowY, diff, cv::CMP_NE);
    EXPECT_EQ(0, cv::countNonZero(diff));
    ::io::removeFile(_testFilePath);
}

TEST_F(FlowIOTestFixture,RandomFlow_FC1) {
    ::io::removeFile(_testFilePath);
    ASSERT_FALSE(io::exists(_testFilePath));
    io::flow::writeFlowFile(_flowX,_flowY,_testFilePath.c_str());
    ASSERT_TRUE(io::exists(_testFilePath));

    cv::Mat_<float> readFlowX, readFlowY;
    std::tie(readFlowX,readFlowY) = io::flow::readFlowFileSplit(_testFilePath.c_str());

    ASSERT_EQ(readFlowX.size(), _flowX.size());
    cv::Mat diff;
    cv::compare(readFlowX, _flowX, diff, cv::CMP_NE);
    EXPECT_EQ(0, cv::countNonZero(diff));

    ASSERT_EQ(readFlowY.size(), _flowY.size());
    cv::compare(readFlowY, _flowY, diff, cv::CMP_NE);
    EXPECT_EQ(0, cv::countNonZero(diff));
    ::io::removeFile(_testFilePath);
}
