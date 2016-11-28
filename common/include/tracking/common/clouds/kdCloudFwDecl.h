//
// Created by bontius on 27/05/16.
//

#ifndef TRACKVIDEO_KDCLOUDFWDECL_H
#define TRACKVIDEO_KDCLOUDFWDECL_H

#include "tracking/common/clouds/typedefsCloud.h"

namespace tracking {

template <typename _CloudT> class KdCloud; // fwDecl
typedef KdCloud<CloudT> KdCloudT;
typedef std::map<FrameId, KdCloudT> KdCloudsT;

} //...ns tracking

#endif //TRACKVIDEO_KDCLOUDFWDECL_H
