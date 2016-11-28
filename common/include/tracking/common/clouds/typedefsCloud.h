//
// Created by bontius on 16/02/16.
//

#ifndef TRACKVIDEO_COMMON_CLOUDTYPEDEFS_H
#define TRACKVIDEO_COMMON_CLOUDTYPEDEFS_H

#include "tracking/common/eigen.h"
#include "soup/geometryTypedefs.h"
#include "tracking/common/typedefs.h"
#include <memory>

namespace Soup {
template <typename _Scalar> class ColoredPointCloud;
} //...ns Soup

namespace tracking {

/** \addtogroup Types
 * @{ */

typedef Soup::ColoredPointCloud<Soup::geometry::Scalar> CloudT;           //!< Main pointcloud type, a pointcloud with normal and color information.
typedef std::shared_ptr<CloudT>       CloudPtrT;        //!< Pointer to cloud with normal and color.
typedef std::shared_ptr<CloudT const> CloudCPtrT;       //!< Const pointer to cloud with normal and color.
typedef std::shared_ptr<CloudT const> CloudConstPtrT;   //!< Const pointer to cloud with normal and color.
typedef std::map<FrameId,CloudCPtrT>  ConstCloudsT;     //!< Pointclouds by time.

/** @} (Types) */

} //...ns tracking

#endif //TRACKVIDEO_COMMON_CLOUDTYPEDEFS_H
