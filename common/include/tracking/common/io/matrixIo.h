//
// Created by bontius on 15/07/16.
//

#ifndef TRACKVIDEO_MATRIXPARSER_H
#define TRACKVIDEO_MATRIXPARSER_H

#include "tracking/common/eigen.h"
#include <fstream>
#include <iostream>

namespace tracking {
namespace io {

template <typename _Scalar, int _Rows, int _Cols>
int matrixFromFile(Eigen::Matrix<_Scalar, _Rows, _Cols> &matrix, std::string const& fileName);


} //...ns common
} //...ns tracking

#endif //TRACKVIDEO_MATRIXPARSER_H
