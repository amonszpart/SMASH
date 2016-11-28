#include "tracking/vis/visualizer.h"
#include "tracking/annot/annotator.h"
#include "tracking/phys/vis/vis.h"

namespace tracking {
  void CallBackFunc( int event, int x, int y, int flags, void* userdata )
  {
      //std::cout << "event: " << event << ", x: " << x << "," << y << std::endl; fflush(stdout);
      tracking::Annotator* annotator = reinterpret_cast<tracking::Annotator*>( userdata );
      if ( annotator )
      {
          annotator->callBackFunc( event, x, y, flags );
          //std::cout << "[" << __func__ << "]: " << "have " << annotator->_rgbs.size() << " images" << std::endl;
      }
  } //...CallBackFunc

  void Annotator::keyPressed( char c )
  {
      bool redraw = false;

      std::cout << "[" << __func__ << "]: " << "key: " << (int)c << std::endl;
      if ( ((c == 's') ) && (_frameId > _frameIds.front() ) )
          backStep(_frameId);
      else if ( ((c == 'w')) && (_frameId+1 <= _frameIds.back()) )
          step(_frameId);
      else if ( ((c == 'd')) )
          upTrackId();
      else if ( (c == 'a') && (_trackId > 0) )
          --_trackId;
      else if ( c == '+' || c == -85 )
          _zoomScale += 1.;
      else if ( ((c == '-') || (c == -83)) && _zoomScale > 1.1 )
          _zoomScale -= 1.;
      else if ( c == 'x' )
      {
          _tracks.erase( _tracks.begin() + _trackId );
      }
      else if ( c == 10 )
          saveTracks(_outPath);
      else if ( c == 'c' )
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              if ( _cuboids.at(_cuboidId).getStates().size() )
              {
                  auto &cuboid = _cuboids.at(_cuboidId);
                  std::multimap< int, FrameId > closest2;
                  for ( auto const& state : cuboid.getStates() )
                  {
                      closest2.insert( std::make_pair(std::abs(int(_frameId)-int(state.first)),state.first) );
                  }
                  while ( closest2.size() > 2 )
                      closest2.erase( closest2.rbegin()->first );
                  //auto it = _cuboids.at(_cuboidId).getStates().lower_bound(_frameId);
                  //if ( it == _cuboids.at(_cuboidId).getStates().end() )
                  //    it = _cuboids.at(_cuboidId).getStates().upper_bound(_frameId);

                  //if ( it == _cuboids.at(_cuboidId).getStates().end() )
                  //    it = _cuboids.at(_cuboidId).getStates().begin();

                  if ( closest2.size() >= 2 )
                  {
                      FrameId prev = closest2.begin()->second;
                      FrameId next = std::next(closest2.begin())->second;
                      if ( next < prev ) std::swap( prev, next );
                      std::cout << "interpolating " << prev << ", and " << next << std::endl;
                      auto state = cuboid.getState(prev);
                      float dt = float(int(_frameId)-int(prev)) / float(int(next)-int(prev));
                      std::cout << "dt: " << dt << " = " << float(int(_frameId)-int(prev)) << " / " << float(int(next)-int(prev)) << std::endl;
                      state.setPose( cuboid.getState(prev).getPose().slerp( dt, cuboid.getState(next).getPose()) );
                      state.setPosition( cuboid.getState(prev).getPosition() + (cuboid.getState(next).getPosition() - cuboid.getState(prev).getPosition()) * dt );
                      cuboid.addState( _frameId, state );
                  } else if ( closest2.size() == 1 )
                      cuboid.addState(_frameId, cuboid.getState(closest2.begin()->second) );
                  else
                      std::cerr << "this can't happen" << std::endl;
              }
              else
                  _cuboids.at( _cuboidId ).addState( _frameId, PoseLoc(Vector3(0.,0.,1.)) );
          }
      }
      else if ( c == -76) // leftArrow
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].rotate( _frameId, Eigen::AngleAxisf( -.5 * M_PI / 180.,
                  _cuboids[ _cuboidId ].getPose(_frameId).toRotationMatrix() * Vector3::UnitZ()
              ).toRotationMatrix() );
          }
          redraw = true;
      }
      else if ( c == -72) // upArrow
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].rotate( _frameId, Eigen::AngleAxisf( -.5 * M_PI / 180.,
                  _cuboids[ _cuboidId ].getPose(_frameId).toRotationMatrix() * Vector3::UnitX()
              ).toRotationMatrix() );
          }
          redraw = true;
      }
      else if ( c == -74) // rightArrow
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].rotate( _frameId, Eigen::AngleAxisf( .5 * M_PI / 180.,
                  _cuboids[ _cuboidId ].getPose(_frameId).toRotationMatrix() * Vector3::UnitZ()
              ).toRotationMatrix() );
          }
          redraw = true;
      }
      else if ( c == -75) // downArrow
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].rotate( _frameId, Eigen::AngleAxisf( .5 * M_PI / 180.,
                  _cuboids[ _cuboidId ].getPose(_frameId).toRotationMatrix() * Vector3::UnitX()
              ).toRotationMatrix() );
          }
          redraw = true;
      }
      else if ( c == -73) // numpad7
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].rotate( _frameId, Eigen::AngleAxisf( -1. * M_PI / 180.,
                  _cuboids[ _cuboidId ].getPose(_frameId).toRotationMatrix() * Vector3::UnitY()
              ).toRotationMatrix() );
          }
          redraw = true;
      }
      else if ( c == -71) // numpad9
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].rotate( _frameId, Eigen::AngleAxisf( 1. * M_PI / 180.,
                  _cuboids[ _cuboidId ].getPose(_frameId).toRotationMatrix() * Vector3::UnitY()
              ).toRotationMatrix() );
          }
          redraw = true;
      }
      else if ( c == 'i' ) // cuboid forward
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].translate( _frameId, Vector3(0.,0.,0.025) );
          }
          redraw = true;
      }
      else if ( c == 'k' ) // cuboid forward
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].translate( _frameId, Vector3(0.,0.,-0.025) );
          }
          redraw = true;
      }
      else if ( c == 'z' ) // cuboid forward
      {
          if ( _cuboidId+1 < CuboidId(_cuboids.size()) )
              ++_cuboidId;
          else
              _cuboidId = 0;
      }
      else if ( c == 49 ) // key '1'
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].scale( Vector3(0.9,1.,1.) );
          }
          redraw = true;
      }
      else if ( c == 50 ) // key '2'
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].scale( Vector3(1.,0.9,1.) );
          }
          redraw = true;
      }
      else if ( c == 51 ) // key '3'
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].scale( Vector3(1.,1.,0.9) );
          }
          redraw = true;
      }
      else if ( c == 52 ) // key '4'
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].scale( Vector3(1.1,1.,1.) );
          }
          redraw = true;
      }
      else if ( c == 53 ) // key '5'
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].scale( Vector3(1.,1.1,1.) );
          }
          redraw = true;
      }
      else if ( c == 54 ) // key '6'
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].scale( Vector3(1.,1.,1.1) );
          }
          redraw = true;
      }
      else if ( c == -79 )
      {
          _mapper.getIntrinsics()(0,0) -= 10;
          std::cout << "intr:\n" << _mapper.getIntrinsics() << std::endl;
      }
      else if ( c == -78 )
      {
          _mapper.getIntrinsics()(0,0) += 10;
          std::cout << "intr:\n" << _mapper.getIntrinsics() << std::endl;
      }
      else if ( c == -80 )
      {
          _mapper.getIntrinsics()(1,1) -= 10;
          std::cout << "intr:\n" << _mapper.getIntrinsics() << std::endl;
      }
      else if ( c == -82 )
      {
          _mapper.getIntrinsics()(1,1) += 10;
          std::cout << "intr:\n" << _mapper.getIntrinsics() << std::endl;
      }
      else if ( c == 'v' )
      {
          _mapper.getIntrinsics()(0,2) -= 5;
          std::cout << "intr:\n" << _mapper.getIntrinsics() << std::endl;
      }
      else if ( c == 'b' )
      {
          _mapper.getIntrinsics()(0,2) += 5;
          std::cout << "intr:\n" << _mapper.getIntrinsics() << std::endl;
      }
      else if ( c == 'n' )
      {
          _mapper.getIntrinsics()(1,2) -= 5;
          std::cout << "intr:\n" << _mapper.getIntrinsics() << std::endl;
      }
      else if ( c == 'm' )
      {
          _mapper.getIntrinsics()(1,2) += 5;
          std::cout << "intr:\n" << _mapper.getIntrinsics() << std::endl;
      }
      else if ( c == 81 )
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].translate( _frameId, Vector3(-0.005,0.,0.) );
          }
          redraw = true;
      }
      else if ( c == 83 )
      {
          if (_cuboidId < CuboidId(_cuboids.size()))
          {
              _cuboids[_cuboidId].translate(_frameId, Vector3(0.005, 0.,0.));
          }
          redraw = true;
      }
      else if ( c == 82 )
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].translate( _frameId, Vector3(0.,-0.005,0.) );
          }
          redraw = true;
      }
      else if ( c == 84 )
      {
          if ( _cuboidId < CuboidId(_cuboids.size()) )
          {
              _cuboids[ _cuboidId ].translate( _frameId, Vector3(0.,0.005,0.) );
          }
          redraw = true;
      }
      else if ( c == 'p' )
      {
          for (size_t k = 0; k != _cuboids.size(); ++k)
          {
              for ( auto const& state : _cuboids.at(_cuboidId).getStates() )
              {
                  if ( _frameId != state.first )
                      continue;
                  const PoseLoc& poseLoc = state.second;
                  std::cout << "pose[" << _frameId << "]:\n" << poseLoc.getPose()                    << std::endl;
                  std::cout << "pose[" << _frameId << "]:\n" << poseLoc.getPose().toRotationMatrix() << std::endl;
                  std::cout << "pos [" << _frameId << "]:\n" << poseLoc.getPosition().transpose   () << std::endl;
              }
          }
      }
      else if ( c == 'g' )
      {
          show3D( true );
      }

      if ( redraw )
      {
          show();
          //show3D( false );
      }
  } //...keyPressed()

  void Annotator::show3D( bool spin )
  {
      if ( !_vis )
      {
          _vis.reset(new Soup::vis::Visualizer<Scalar>("3D view"));
          _vis->addCoordinateSystem(1.0);
      }
      else
          _vis->removeActorsByPrefix( "cuboid" );

      int cuboidId(0);
      for ( auto const& cuboid : _cuboids )
      {
          ++cuboidId;
          drawCuboidStates(cuboid, *_vis, 1., Vector3(.8, .2, 0.), cuboidId, _frameId, &_frameIds );
      }
      if ( spin )
          _vis->spin();
      else
          _vis->spinOnce(5);
  }

  void Annotator::addPoint( int x, int y )
  {
      auto &track( _tracks.getTrack(_trackId) );
      if ( track.hasPoint( _frameId) )
          track.getPoint(_frameId) = tracking::TrackPoint2D(x,y);
      else
          track.addPoint(_frameId,tracking::TrackPoint2D(x,y));
      // next frame
      keyPressed( 82 );
  }

  void Annotator::show()
  {
      if (!_inited)
      {
          init();
          _inited = true;
      }
      cv::Mat rgb = _rgbs.at(_frameId - _frameIds.front()).clone();

      // draw tracks
      TrackId i(0u);
      for ( auto const& track : _tracks )
      {
          const tracking::TrackPoint2D* prev( nullptr );
          float radius = 1.5, thickness = 1.;
          if ( i == _trackId )
              radius += 1.f;

          for ( auto const& frameIdAndTrackPoint : track.getPoints() )
          {
              thickness = 1.;
              const auto &frameId   ( frameIdAndTrackPoint.first  );
              const auto &trackPoint( frameIdAndTrackPoint.second );
              cv::Point cvPoint( trackPoint.getPoint()(0),trackPoint.getPoint()(1) );
              cv::Scalar color = _colors.at( i % _colors.size() );
              if ( frameId != _frameId )  color =  cv::Scalar(0,0,0);
              else                        { color += cv::Scalar(100,50,50); thickness += 1.; }
              cv::circle( rgb, cvPoint, radius, color, thickness );
              if ( prev )
              {
                  if ( frameId == _frameId )
                      color -= cv::Scalar(100,100,50);
                  cv::line( rgb, cv::Point(prev->getPoint()(0),prev->getPoint()(1)), cvPoint, color );
              }
              prev = &trackPoint;
          }
          ++i;
      }
      cv::circle( rgb, _zoomPoint, 3, _colors.at(_trackId % _colors.size()), 1 );

      // draw shapes
      for ( auto const& cuboid : _cuboids )
      {
          if ( cuboid.hasFrame(_frameId) )
          {
              drawCuboid(cuboid.getTransformation(_frameId), rgb, _mapper );
          }
      }

      char text[256];
      sprintf( text, "im%d,tr%d", _frameId, _trackId );
      cv::putText( rgb, text, cv::Point(10,40), 1, 2, cv::Scalar(255,255,255,255), 1 );
      cv::imshow( _winName, rgb );

#if 1
      const cv::Size halfCropSize(_zoomScale * 50,_zoomScale * 50);
      cv::Point2i tmpZoomPoint(
          std::min( rgb.cols - halfCropSize.width , std::max(halfCropSize.width,static_cast<int>(std::round(_zoomPoint.x))) ),
          std::min( rgb.rows - halfCropSize.height, std::max(halfCropSize.height,static_cast<int>(std::round(_zoomPoint.y))) ));
      cv::Rect rect( tmpZoomPoint.x - halfCropSize.width,
          tmpZoomPoint.y - halfCropSize.height,
          2*halfCropSize.width,
          2*halfCropSize.height );

      cv::Mat zoomed;
//        std::cout << "rect: " << rect.x << "," << rect.y << "," << rect.width << "," << rect.height
//                  << ", rgb:  " << rgb.cols << "," << rgb.rows
//                  << ", scale: " << _zoomScale << ", zoompoint:"
//                  << _zoomPoint.x << "," << _zoomPoint.y << std::endl;
      cv::resize( rgb(rect), zoomed, cv::Size(rect.width*_zoomScale,rect.height*_zoomScale) );
      cv::Mat cropped = zoomed;
#else
      cv::Mat zoomed;
        cv::resize(rgb, zoomed, cv::Size(rgb.cols*_zoomScale,rgb.rows*_zoomScale) );
        const cv::Size halfCropSize(_zoomScale * 50,_zoomScale * 50);
        cv::Point2i tmpZoomPoint( std::min( zoomed.cols - halfCropSize.width , std::max(halfCropSize.width ,static_cast<int>(std::round(_zoomPoint.x * _zoomScale))) ),
            std::min( zoomed.rows - halfCropSize.height, std::max(halfCropSize.height,static_cast<int>(std::round(_zoomPoint.y * _zoomScale))) ));
        cv::Rect rect( tmpZoomPoint.x - halfCropSize.width, tmpZoomPoint.y - halfCropSize.height,
            2 * halfCropSize.width, 2 * halfCropSize.height );
        cv::Mat cropped = zoomed( rect );
#endif
      //std::cout << "rect: " << rect << std::endl; fflush(stdout);
      //cv::line( zoomed, cv::Point(tmpZoomPoint.x - 5, tmpZoomPoint.y - 5),cv::Point(tmpZoomPoint.x + 5, tmpZoomPoint.y + 5), cv::Scalar(255,255,0), 2, 3 );
      //cv::line( zoomed, cv::Point(tmpZoomPoint.x + 5, tmpZoomPoint.y - 5),cv::Point(tmpZoomPoint.x - 5, tmpZoomPoint.y + 5), cv::Scalar(255,255,0), 2, 3 );
      cv::imshow( "zoom", cropped );
  } //...Annotator::show()

  void Annotator::init()
  {
      cv::namedWindow(_winName);
      cv::setMouseCallback( _winName,CallBackFunc,static_cast<void*>(this) );
      cv::namedWindow("zoom");
      cv::moveWindow("zoom",640,0);
      _frameId = _frameIds.front();
      _trackId = -1;
      upTrackId();
      //_colors = colors::nColoursCv<cv::Scalar>( 16, 255., true );
      auto colors = colors::paletteMediumColoursEigen2( 16, 255. );
      for ( auto const& color : colors )
      {
          _colors.push_back( cv::Scalar(color(0),color(1),color(2)) );
      }
      if ( _cuboids.size() )
          _frameId = _cuboids.at(0).getStates().begin()->first;
      else
          _frameId = (_frameIds.back() - _frameIds.front() + 1) / 4 + _frameIds.front();
  }

  void Annotator::upTrackId()
  {
      if ( (_trackId != TrackId(-1)) && (_trackId == _tracks.getTrackCount() - 1 && _tracks.getTrackCount() && !_tracks.back().size()) )
      {
          std::cout << "_tracks.getTrackCount() : " << _tracks.getTrackCount() << ", ";
          if ( _tracks.getTrackCount() )
              std::cout << ", _tracks.back().size(): " << _tracks.back().size() << std::endl;
          return;
      }

      ++_trackId;

      if ( _trackId >= _tracks.getTrackCount() )
      {
          _tracks.addTrack( tracking::Track2D(_trackId) );
      }
  }

  void Annotator::saveTracks( const std::string outPath )
  {
      using namespace tracking;

      tracking::SequentialWriter writer( outPath, _rgbs.size() );
      for ( auto const& track : _tracks )
      {
          if ( track.size() )
              writer.writeNext( track );
      }
      std::cout << "wrote to " << outPath << std::endl;

#if 0
      // save sift files
                {
                    for ( FrameId frameId = 0; frameId != _rgbs.size(); ++frameId )
                    {
                        std::vector< tracking::TrackPoint2D > points;
                        points.reserve( _tracks.size() );
                        for ( auto const& track: _tracks )
                        {
                            if ( track.hasPoint( _frameId ))
                            {
                                points.push_back( track.getPoint(_frameId) );
                            }
                        }

                        char name[255];
                        sprintf( name, "color_%05u.sift", frameId );
                        std::cout << "opening " << name << std::endl;
                        std::ofstream f( name );
                        f << points.size() << " 128\n";
                        for ( auto const& point : points )
                        {
                            f << point(0) << " " << point(1) << " 0 0";
                            for ( int j = 0; j != 128; ++j )
                                f << " " << 0;
                            f << "\n";
                        }
                    } //...for all frames
                } //...sift writing
#endif
      tracking::bundle_physics::io::writeCuboids( _cuboids, "cuboids.json" );
  } //...saveTracks()

  void Annotator::callBackFunc( int event, int x, int y, int flags )
  {
      if ( flags & cv::EVENT_FLAG_SHIFTKEY )
      {
          if ( event == cv::EVENT_LBUTTONUP )
          {
              if ( _cuboidId < CuboidId(_cuboids.size()) && _cuboids[_cuboidId].hasFrame(_frameId) )
              {
                  auto pnt3 = _cuboids[ _cuboidId ].getPosition(_frameId);
                  auto pnt2  = _mapper.to2D( pnt3, 0 );
                  pnt2(0) = x;
                  pnt2(1) = y;
                  _cuboids[ _cuboidId ].setPosition( _frameId, _mapper.to3D(pnt2.template head<2>().eval(),pnt3(2),0) );
              }
              else
              {
                  std::cout << "cuboid " << _cuboidId << " does not have frame " << _frameId << std::endl;
                  for ( auto const& state : _cuboids[_cuboidId].getStates() )
                      std::cout << "state[" << state.first << "]: " << state.second.getPosition().transpose() << std::endl;
              }
          }
      }
      else
      {
          if ( event == cv::EVENT_LBUTTONUP )
          {
              addPoint( x, y );
          }//// ============================= Parabola ============================= ////
          else if ( (event == cv::EVENT_MOUSEMOVE) )
          {
              _zoomPoint.x = x;
              _zoomPoint.y = y;
          }
      }

      show();
  }

  void Annotator::addCuboid( const Cuboid& cuboid )
  {
      _cuboids.push_back( cuboid );
  }
} //...ns tracking
