#ifndef TV_TRACK_HPP
#define TV_TRACK_HPP

#include "tracking/common/track.h"

namespace tracking {
  template <typename _PointT>
  inline typename _PointT::Scalar distance(const _PointT& a, const _PointT& b) {
      return (a.getPoint() - b.getPoint()).norm();
  } //...distance()

  template <typename _PointT>
  inline void TrackTemplate<_PointT>::addPoint(FrameId frame, _PointT const& point) {
      if ( _points.find(frame) != _points.end() )
          std::cerr << "[" << __func__ << "]: " << "Already has frame " << frame << std::endl;
      //_points.insert( std::make_pair(frame,point) );
      _points[ frame ] = point;
  } //...addPoint()

  template <typename _PointT>
  inline void TrackTemplate<_PointT>::addPoint(FrameId frame, _PointT&& point) {
      if (_points.find(frame) != _points.end())
          std::cerr << "[" << __func__ << "]: " << "Already has frame " << frame << std::endl;
      //_points.insert( std::make_pair(frame,point) );
      _points.emplace(frame,std::move(point));
  } //...addPoint()

  // for now, distance of first points
  template <typename _PointT>
  inline typename TrackTemplate<_PointT>::Scalar
  TrackTemplate<_PointT>::distance( const TrackTemplate<_PointT>& t0, const TrackTemplate<_PointT>& t1 )
  {
      const int nSteps = 5;
      const int step = std::max(1LU,t0.getPoints().size() / nSteps);
      Scalar minDist = std::numeric_limits<Scalar>::max();

      for ( size_t i = 0; i < t0.getPoints().size(); i += step )
      {
          auto it0 = t0.getPoints().begin();
          std::advance( it0, i );
          const _PointT& pnt0 = it0->second;
          FrameId frame = it0->first;
          if ( t1.hasPoint(frame) )
          {
              Scalar dist = tracking::distance( pnt0, t1.getPoint(frame) ); // (pnt0.p - t1.getPoint(frame).p).norm()
              //Scalar dist = ( pnt0 - t1.getPoint(frame) ).norm(); // (pnt0.p - t1.getPoint(frame).p).norm()
              if ( minDist > dist )
                  minDist = dist;
          }

          if ( i < t1.getPoints().size() )
          {
              auto it1 = t1.getPoints().begin();
              std::advance( it1, i );
              const _PointT &pnt1    = it1->second;
              const FrameId  frameId = it1->first;
              if (t0.hasPoint(frameId)) {
                  Scalar dist = (pnt1.getPoint() - t1.getPoint(frameId).getPoint()).norm();
                  if ( minDist > dist )
                      minDist = dist;
              }
          }
      }
      return minDist;
  } //..distance()

  template <typename _PointT>
  inline typename _PointT::LocationT TrackTemplate<_PointT>::getCentroid() const
  {
      typedef typename _PointT::LocationT LocationT;
      LocationT pnt( LocationT::Zero() );
      for ( auto const& point : this->getPoints() )
          pnt += point.second.getPoint();
      if ( this->size() )
          pnt /= static_cast< typename _PointT::Scalar >( this->size() );
      return pnt;
  } //...getCentroid()

  template <typename _PointT>
  template <typename _CloudT>
  inline typename _CloudT::PtrT
  TrackTemplate<_PointT>::getCloud() const {
      typename _CloudT::PtrT cloudPtr( new _CloudT(this->size()) );
      int pId( 0 );
      for (auto const& points : this->getPoints()) {
          cloudPtr->getPoint(pId) = points.second.getPoint();
          if (_PointT::RowsAtCompileTime > 3)
              cloudPtr->getNormal(pId) = points.second.getNormal();
          //std::cout << "[" << __func__ << "]: " << "point[" << pId << "]: " << cloudPtr->getPoint(pId) << " vs " << points.second.transpose() << std::endl;
          ++pId;
      }
      return cloudPtr;
  } //...getCloud()

  template <typename _PointT>
  inline typename TrackTemplate<_PointT>::Scalar TrackTemplate<_PointT>::getLength() const
  {
      Scalar        displacement( 0.      );
      const PointT *prev        ( nullptr );
      for (auto it = this->getPoints().begin(); it != this->getPoints().end(); ++it) {
          if (prev)
              displacement += (it->second.getPoint() - prev->getPoint()).norm();
          prev = &it->second;
      }
      return displacement;
  } //...getLength()

  template <typename _PointT>
  inline typename TrackTemplate<_PointT>::LabelT      & TrackTemplate<_PointT>::getLabel()
  { return mLabel; }

  template <typename _PointT>
  inline typename TrackTemplate<_PointT>::LabelT const& TrackTemplate<_PointT>::getLabel() const
  { return mLabel; }

  template <typename _PointT>
  inline TrackId TrackTemplate<_PointT>::getTrackId() const {
      if ( mLabel < 0 )
          throw new TrackTemplate_TrackIdNegativeException("");
      return static_cast<TrackId>(mLabel);
  }

  template <typename _PointT>
  inline void TrackTemplate<_PointT>::setLabel(TrackTemplate::LabelT const label)
  { mLabel = label; }

  template <typename _PointT>
  inline typename TrackTemplate<_PointT>::PointsByFrame const& TrackTemplate<_PointT>::getPoints() const
  { return _points; }

  template <typename _PointT>
  inline bool TrackTemplate<_PointT>::hasPoint(FrameId frame) const
  { return _points.find(frame) != _points.end(); }

  template <typename _PointT>
  inline _PointT const& TrackTemplate<_PointT>::getPoint(FrameId frame) const
  { return _points.at(frame); }

  template <typename _PointT>
  inline _PointT& TrackTemplate<_PointT>::getPoint(FrameId frame)
  { return _points.at(frame); }

  template <typename _PointT>
  inline typename TrackTemplate<_PointT>::PointsByFrame::iterator
  TrackTemplate<_PointT>::erase(typename PointsByFrame::iterator it)
  { return _points.erase(it); }

  template <typename _PointT>
  inline typename TrackTemplate<_PointT>::PointsByFrame::const_iterator
  TrackTemplate<_PointT>::find_if(FrameId hintFrameId, std::function<bool (typename PointsByFrame::value_type const&)> f) const {
      auto const iter = _points.find(hintFrameId);
      if (iter != _points.end() && f(*iter))
          return iter;
      else
          return std::find_if(_points.begin(), _points.end(), f);
  } //...find_if()

} //...ns tracking

#endif // TV_TRACK_HPP
