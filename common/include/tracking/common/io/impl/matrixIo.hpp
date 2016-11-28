//
// Created by bontius on 15/07/16.
//

#ifndef TRACKVIDEO_MATRIXPARSER_HPP
#define TRACKVIDEO_MATRIXPARSER_HPP

#include "tracking/common/io/matrixIo.h"

namespace tracking {
namespace io {

template<typename _Scalar, int _Rows, int _Cols>
inline int matrixFromFile(Eigen::Matrix<_Scalar, _Rows, _Cols> &matrix, std::string const& fileName) {
    std::ifstream fin(fileName.c_str());
    if (!fin.is_open()) {
        std::cerr << "Could not open " << fileName << std::endl;
        return EXIT_FAILURE;
    }
    static_assert(_Rows != Eigen::Dynamic, "Can't handle dynamic matrices (_Rows==-1)");
    static_assert(_Cols != Eigen::Dynamic, "Can't handle dynamic matrices (_Cols==-1)");
    static_assert(_Rows + _Cols > 3, "Matrix can't have less, than 2 entries...");
    matrix = Eigen::Matrix<_Scalar,_Rows,_Cols>::Zero();
    _Scalar tmp(0.);
    for (int row = 0; row != _Rows; ++row)
        for (int col = 0; col != _Cols; ++col) {
            fin >> tmp;
//            fin >> matrix.coeffRef(row, col);
            matrix(row,col) = tmp;
            std::cout << "[" << __func__ << "] " << "parsed " << row << "," << col << ": " << matrix.coeff(row,col) << ", from " << fileName << std::endl;
        }
    return EXIT_SUCCESS;
} //...matrixFromFile()

} //...ns common
} //...ns trackign

#endif //TRACKVIDEO_MATRIXPARSER_HPP
