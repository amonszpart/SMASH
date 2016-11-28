#include "tracking/common/trackPoint3d.h"
#include "tracking/common/trackPoint2d.h"
#include "tracking/common/impl/track.hpp"
#include "soup/cloudTypedefs.h"
#include "soup/pointCloudEigen.h"
#include <ostream>
#include <iostream>

std::ostream& operator<<(std::ostream& os, const tracking::Track2D& track)
{
    os << "l" << track.getLabel() << ", " << track.getPoints().size() << "(";
    for ( auto const& pair : track.getPoints() )
    {
        os << pair.second.getPoint()(0) << "," << pair.second.getPoint()(1) << "; ";
    }
    os << ")";
    return os;
}

//std::ostream& operator<<(std::ostream& os, const tracking::Point2& point)
//{
//    os << point.x[0] << "," << point.x[1];
//    return os;
//}

namespace _track_h {
    typedef Soup::geometry::Scalar Scalar;
    typedef Soup::geometry::ColoredCloudPtrT ColoredCloudPtrT;
    typedef Soup::geometry::ColoredCloudT ColoredCloudT;
    //typedef Soup::geometry::CloudT CloudT;
    //typedef Soup::geometry::CloudPtrT CloudPtrT;
}

template
_track_h::ColoredCloudPtrT tracking::TrackTemplate<tracking::TrackPoint3D>::getCloud<_track_h::ColoredCloudT>() const;
//template _track_h::CloudPtrT        tracking::TrackTemplate<tracking::TrackPoint3D>::getCloud<_track_h::CloudT>() const;

template tracking::TrackPoint2D::Scalar tracking::distance( const TrackPoint2D& a, const TrackPoint2D& b );
template tracking::TrackPoint3D::Scalar tracking::distance( const TrackPoint3D& a, const TrackPoint3D& b );

template class tracking::TrackTemplate<tracking::TrackPoint2D>;
template class tracking::TrackTemplate<tracking::TrackPoint3D>;
