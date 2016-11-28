//
// Created by bontius on 29/12/15.
//

#ifndef TRACKVIDEO_PHYS_CUBOIDIO_H
#define TRACKVIDEO_PHYS_CUBOIDIO_H

#include <vector>
#include <string>

namespace tracking {
  namespace bundle_physics {
    namespace io {
      template <typename _CuboidT>
      void writeCuboids(const std::vector<_CuboidT> &cuboids, const std::string &path, typename _CuboidT::Scalar cSqr = 0.);

      template <typename _CuboidT>
      int  readCuboids(std::vector<_CuboidT> &cuboids, const std::string &inPath);

      template <typename _CuboidT>
      int  readCuboids(std::vector<_CuboidT> &cuboids, const std::vector<std::string> &inPaths);
    }//...ns io
  } //...ns bundle_physics
}//...ns tracking
#endif //TRACKVIDEO_PHYS_CUBOIDIO_H
