//
// Created by bontius on 02/06/16.
//

#ifndef TRACKVIDEO_COMMON_EIGENUTIL_H
#define TRACKVIDEO_COMMON_EIGENUTIL_H

#include "tracking/common/eigen.h"

template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
    return ( (x - x).array() == (x - x).array()).all();
}


template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
    return !((x.array() == x.array())).all();
}

#endif //TRACKVIDEO_COMMON_EIGENUTIL_H
