//
// Created by bontius on 28/01/16.
//

#include "tracking/common/util/util.h"
#include "tracking/common/util/impl/util.hpp"

namespace tracking {
    template std::string string_cast( float     const& value );
    template std::string string_cast( double    const& value );
    template std::string string_cast( int       const& value );
    template std::string string_cast( unsigned  const& value );
    template std::string string_cast( unsigned long const& value );
}