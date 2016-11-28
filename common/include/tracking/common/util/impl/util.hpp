//
// Created by bontius on 28/01/16.
//

#ifndef TRACKVIDEO_UTIL_HPP
#define TRACKVIDEO_UTIL_HPP

#include <sstream>
#include "tracking/common/util/util.h"

namespace tracking {

    // http://bytes.com/topic/c/answers/440963-creating-stringstream-parameter
    template <class _ValueT>
    inline std::string string_cast( const _ValueT& value )
    {
        std::stringstream stream;
        stream << value;
        return stream.str();
    } //...string_cast()

} //...ns tracking

#endif //TRACKVIDEO_UTIL_HPP
