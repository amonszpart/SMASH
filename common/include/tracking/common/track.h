#ifndef TRACKVIDEO_COMMON_TRACK_H
#define TRACKVIDEO_COMMON_TRACK_H

//#include <Eigen/StdVector>
//EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Vector2f);

#include "tracking/common/eigen.h"
#include "tracking/common/trackFwDecl.h"
#include "tracking/common/trackPoint2d.h"
#include "tracking/common/trackPoint3d.h"
#include "tracking/common/util/exception.h"
#include "tracking/common/typedefs.h"
#include "soup/geometryTypedefs.h"
#include "tracking/common/clouds/typedefsCloud.h"
#include <memory>
#include <map>
#include <vector>
#include <set>
#include <array>
#include <iostream>
#include <ostream>

namespace tracking {
  DEFINE_EXCEPTION(TrackTemplate_TrackIdNegative)

  template <typename _PointT>
  class TrackTemplate {
          enum { NeedsToAlign = (sizeof(_PointT)%16)==0 };
      public:
          typedef          _PointT             PointT;
          typedef typename _PointT::Scalar     Scalar;
          typedef std::map<FrameId,_PointT>    PointsByFrame;
          typedef int                          LabelT;

          TrackTemplate() : mLabel(-1) {}
          explicit TrackTemplate(LabelT const label) : mLabel(label) {}
          explicit TrackTemplate(TrackId const trackId) : mLabel(static_cast<LabelT>(trackId)) {}

          LabelT&                               getLabel     ();
          LabelT const&                         getLabel     ()              const;
          TrackId                               getTrackId   () const;
          void                                  setLabel     (LabelT const label );
          PointsByFrame const&                  getPoints    ()              const;
          void                                  addPoint     (FrameId frame, const _PointT& point );
          void                                  addPoint     (FrameId frame, _PointT&& point);

          bool                                  hasPoint     (FrameId frame) const;
          _PointT const&                        getPoint     (FrameId frame) const;
          _PointT&                              getPoint     (FrameId frame);
          inline typename PointsByFrame::const_iterator         findPoint    (FrameId frame) const {return _points.find(frame);}
          inline typename PointsByFrame::iterator               findPoint    (FrameId frame)       {return _points.find(frame);}
          inline          _PointT const&                        getFirstPoint()              const { return _points.begin()->second; }
          /*  */ typename _PointT::LocationT                    getCentroid  ()              const;
          inline          void                                  clear        ()                    { mLabel = 0; _points.clear(); }
          inline          size_t                                size         ()              const { return _points.size(); }
          inline typename PointsByFrame::const_iterator         begin        ()              const { return _points.begin(); }
          inline typename PointsByFrame::const_iterator         end          ()              const { return _points.end(); }
          inline typename PointsByFrame::iterator               begin        ()                    { return _points.begin(); }
          inline typename PointsByFrame::iterator               end          ()                    { return _points.end(); }
          inline typename PointsByFrame::const_reverse_iterator rbegin       ()              const { return _points.rbegin(); }
          inline typename PointsByFrame::const_reverse_iterator rend         ()              const { return _points.rend(); }
          inline typename PointsByFrame::reverse_iterator       rbegin       ()                    { return _points.rbegin(); }
          inline typename PointsByFrame::reverse_iterator       rend         ()                    { return _points.rend(); }
          typename PointsByFrame::iterator      erase(typename PointsByFrame::iterator it);

          typename PointsByFrame::const_iterator
          find_if(FrameId hintFrameId, std::function<bool(typename PointsByFrame::value_type const& p)> f) const;

          /* */           Scalar                                distance     (const TrackTemplate& t0, const TrackTemplate& t1);

          /** \brief Gathers points to a compact cloud using data duplication. */
          template <typename _CloudT>
          /* */  typename _CloudT::PtrT                         getCloud     ()              const;

          Scalar                                                getLength() const;
      protected:
          LabelT        mLabel;
          PointsByFrame _points;
      public:
          EIGEN_MAKE_ALIGNED_OPERATOR_NEW_IF(NeedsToAlign)
  }; //...cls Track
} //...ns tracking()

std::ostream& operator<<(std::ostream& os, const tracking::Track2D& track);

#endif // TRACKVIDEO_COMMON_TRACK_H



