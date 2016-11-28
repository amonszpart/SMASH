//
// Created by bontius on 15/07/16.
//

#include "tracking/common/io/impl/matrixIo.hpp"

template int tracking::io::matrixFromFile(Eigen::Matrix<float,3,3> &matrix, std::string const& fileName);
template int tracking::io::matrixFromFile(Eigen::Matrix<double,3,3> &matrix, std::string const& fileName);
template int tracking::io::matrixFromFile(Eigen::Matrix<float,4,4> &matrix, std::string const& fileName);
template int tracking::io::matrixFromFile(Eigen::Matrix<double,4,4> &matrix, std::string const& fileName);
