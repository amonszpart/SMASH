//
// Created by bontius on 05/02/16.
//

#ifndef SOUP_CLOUDTYPEDEFS_H
#define SOUP_CLOUDTYPEDEFS_H

#include "soup/geometryTypedefs.h"
#include <memory>

namespace Soup {
template <typename _Scalar> class ColoredPointCloud;
namespace geometry {
/** \addtogroup Types
 * @{ */

//typedef PointCloud <Scalar>                CloudT;             //!< \deprecated because of ambiguity
//typedef typename CloudT::Ptr               CloudPtrT;          //!< \deprecated because of ambiguity
//typedef typename CloudT::ConstPtr          CloudConstPtrT;     //!< \deprecated because of ambiguity
//    typedef PointCloud <Scalar>                PointCloudT;
//    typedef typename PointCloudT::Ptr          PointCloudPtrT;

typedef ColoredPointCloud<Scalar>           ColoredCloudT;
typedef std::shared_ptr<ColoredCloudT>       ColoredCloudPtrT;
typedef std::shared_ptr<const ColoredCloudT> ColoredCloudConstPtrT;
//    typedef typename ColoredCloudT::ConstPtrsT ColoredCloudPtrsT;
//    typedef typename ColoredCloudT::PtrsT      CloudsT;
//    typedef typename ColoredCloudT::ConstPtrsT ConstCloudsT;

/** @} (Types) */

} //...ns geometry
} //...ns Soup
#endif //SOUP_CLOUDTYPEDEFS_H
