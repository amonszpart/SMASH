//
// Created by bontius on 29/12/15.
//

#include "tracking/phys/io/cuboidIo.h"
#include "tracking/phys/io/impl/cuboidIo.hpp"
#include "tracking/annot/cuboid.h"
#include <string>
#include <fstream>
#include <vector>


namespace tracking {
  namespace bundle_physics {
    namespace io {
      template
      void writeCuboids(std::vector<Cuboid> const&cuboids, std::string const& path, Cuboid::Scalar cSqr);
      template
      int readCuboids(std::vector<Cuboid> &cuboids, const std::string &inPath);
      template
      int readCuboids(std::vector<Cuboid> &cuboids, const std::vector<std::string> &inPaths);
    } //...ns io
  } //...ns bundle_physics
} //...ns tracking