//
// Created by bontius on 29/03/16.
//

#ifndef TRACKVIDEO_PHYSFUNCTORINFOFWDECL_H
#define TRACKVIDEO_PHYSFUNCTORINFOFWDECL_H

#include <list>
#include <memory>

namespace tracking {
  namespace bundle_physics {
    class PhysFunctorInfo;
    typedef std::list<std::shared_ptr<PhysFunctorInfo> > PhysFunctorInfosT;
  } //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_PHYSFUNCTORINFOFWDECL_H
