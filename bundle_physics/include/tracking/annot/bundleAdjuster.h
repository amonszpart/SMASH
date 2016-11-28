#ifndef TV_BUNDLEADJUSTER_H
#define TV_BUNDLEADJUSTER_H

#include "Eigen/Core"
#include "opencv2/core/core.hpp"
#include "tracking/common/correspondence.h"
#include "tracking/common/mapper.h"
#include "tracking/common/optParams.h"
#include "tracking/common/tracks.h"
#include "tracking/annot/cuboid.h"
#include "soup/cloudTypedefs.h"
#include "soup/geometryTypedefs.h"

namespace tracking {
    class BundleAdjuster {
            typedef double                                          CeresScalar;
            typedef tracking::FrameId                               FrameId;
            typedef tracking::TrackId                               TrackId;
            typedef Eigen::Matrix<CeresScalar,3,3>                  Matrix3d;
            typedef Soup::geometry::Scalar                          Scalar;
            typedef Soup::geometry::HelperTypes<Scalar>::Transform  TransformationT;
            typedef Soup::geometry::HelperTypes<Scalar>::Vector3    Vector3;
//            typedef Soup::geometry::CloudsT                         CloudsT;
            typedef typename CloudsT::value_type                    CloudPtrT;
            typedef typename CloudPtrT::element_type                CloudT;

        public:
            static constexpr int CAMERA_STRIDE = 6;
            static constexpr int POINT_STRIDE  = 3;

            struct Indexer
            {
                    Indexer( const FrameId &startFrameId, const TrackId& trackCount )
                        : _startFrameId( startFrameId ), _trackCount( trackCount )
                    {}

                    inline CeresScalar* getCamera( CeresScalar* unknowns, const FrameId frameId ) { return unknowns + (frameId - _startFrameId) * CAMERA_STRIDE; }
                    inline CeresScalar* getPoint ( CeresScalar* unknowns, const TrackId trackId ) { return unknowns + trackId                   * POINT_STRIDE;  }
                protected:
                    FrameId _startFrameId;
                    TrackId _trackCount;
            }; //...Indexer

            static int main( tracking::Tracks3D& tracks3d,
                             TransformsT &outTransforms,
                             const tracking::Tracks2D& tracks2d,
                             const Mapper &mapper,
                             const FrameId startFrame,
                             const FrameId endFrame,
                             const OptParams& optParams,
                             const std::vector< ::Eigen::Vector3f >& colors,
                             const std::vector< cv::Mat >          & rgbs,
                             TransformsT       * inTransforms = nullptr
                             );
    }; //...cls BundleAdjuster
} //...ns tracking

#endif // TV_BUNDLEADJUSTER_H
