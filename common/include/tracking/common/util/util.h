//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_UTIL_H
#define TRACKVIDEO_UTIL_H

#include <string>

namespace tracking {
    template <class _ValueT>
    std::string string_cast( const _ValueT& value );

    inline double randd(double const scale = double(1.0)) { return rand() / double(RAND_MAX) * scale; }
    inline float  randf(float  const scale = float (1.f)) { return rand() / float (RAND_MAX) * scale; }
} //...ns tracking

#endif //TRACKVIDEO_UTIL_H
