//
// Created by bontius on 28/03/16.
//

#ifndef TRACKVIDEO_WEIGHTSIO_H
#define TRACKVIDEO_WEIGHTSIO_H

#include <utility>
//#include "tracking/phys/weights.h"

namespace tracking {
  namespace bundle_physics {
    class Weights;
    namespace io {
      std::pair<Weights,int> readWeights(int argc, const char **argv);
    } //...ns io
  } //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_WEIGHTSIO_H
