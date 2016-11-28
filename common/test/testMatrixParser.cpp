//
// Created by bontius on 15/07/16.
//

#include "tracking/common/io/matrixIo.h"
#include "tracking/common/io/os.h"
#include "tracking/common/util/util.h"

#include <gtest/gtest.h>

TEST(DISABLED_MatrixParser, Read3x3) {
    Eigen::Matrix3d mx;
    for (int i = 0; i != mx.rows() * mx.cols(); ++i)
        mx(i) = 100 * tracking::randf() - 50.f;
    for (int row = 0; row != mx.rows(); ++row)
        for (int col = 0; col != mx.cols(); ++col)
            EXPECT_NE(mx(row,col), 0.);

    std::string const _testFilePath {"data/testIntr.txt"};
    ::io::removeFile(_testFilePath);
    ASSERT_FALSE(::io::exists(_testFilePath));
    std::ofstream fout(_testFilePath);
    ASSERT_TRUE(fout.is_open());

    for (int row = 0; row != mx.rows(); ++row)
        for (int col = 0; col != mx.cols(); ++col) {
            fout << mx(row, col);
            if (row * mx.cols() + col != mx.rows() * mx.cols() - 1)
                fout << " ";
        }
    Eigen::Matrix3d readMx;
    EXPECT_EQ(tracking::io::matrixFromFile(readMx,_testFilePath), EXIT_SUCCESS);
    for (int row = 0; row != mx.rows(); ++row)
        for (int col = 0; col != mx.cols(); ++col) {
            EXPECT_NEAR(mx(row,col), readMx(row,col), std::numeric_limits<double>::epsilon());
        }

}
