//
// Created by bontius on 03/04/16.
//

#ifndef TRACKVIDEO_CERESOPTIONSIO_H
#define TRACKVIDEO_CERESOPTIONSIO_H

#include "tracking/common/util/impl/parse.hpp"

namespace tracking {
  namespace io {
    template <typename _Options>
    void readCeresOptions( int argc, const char** argv, _Options &options ) {
        console::parse_arg(argc, argv, "--function-tol", options.function_tolerance);
        console::parse_arg(argc, argv, "--parameter-tol", options.parameter_tolerance);
        console::parse_arg(argc, argv, "--max-time", options.max_solver_time_in_seconds);
        console::parse_arg(argc, argv, "--max-iterations", options.max_num_iterations);
  } //...readCeresOptions()
}//...ns io
} //...ns tracking

#endif //TRACKVIDEO_CERESOPTIONSIO_H
