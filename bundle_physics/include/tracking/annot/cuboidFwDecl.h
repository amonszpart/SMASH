//
// Created by bontius on 28/03/16.
//

#ifndef TRACKVIDEO_CUBOIDFWDECL_H
#define TRACKVIDEO_CUBOIDFWDECL_H

#include "tracking/phys/typedefs.h"
#include "tracking/common/typedefs.h"
#include <vector>
#include <memory>

namespace tracking {
  namespace bundle_physics {
    class Cuboid;
    typedef std::vector     <Cuboid>            CuboidsT;
    typedef std::shared_ptr <CuboidsT>          CuboidsPtrT;
    typedef std::shared_ptr <const CuboidsT>    CuboidsConstPtrT;
    typedef std::map        <GroupId, CuboidId> GroupsCuboidsT;

  } //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_CUBOIDFWDECL_H
