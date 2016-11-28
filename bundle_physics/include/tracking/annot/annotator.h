#ifndef TV_ANNOTATOR_H
#define TV_ANNOTATOR_H

#include "tracking/annot/cuboid.h"
#include "tracking/phys/io/cuboidIo.h"
#include "tracking/phys/typedefsGeometry.h"
#include "tracking/phys/typedefs.h"
#include "tracking/common/util/colors.h"
#include "tracking/common/io/sequentialReader.h"
#include "tracking/common/typedefs.h"
#include "tracking/vis/visualizer.h"
#include "picojson/picojsonUtil.h"
#include "picojson/picojson.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <string>
#include <vector>
#include <iostream>
#include <fstream>

namespace tracking {
  using Cuboid = bundle_physics::Cuboid;
  using Scalar = bundle_physics::Scalar;
  using Vector3 = bundle_physics::Vector3;
  using PoseLoc = bundle_physics::PoseLoc;

  void CallBackFunc( int event, int x, int y, int flags, void* userdata );

  class Annotator
  {
      public:
          Annotator( std::string winName, LinRgbsT& rgbs, std::string outPath, const tracking::Mapper &mapper, const FrameIdsT& frameIds, const tracking::Tracks2D* tracks = nullptr )
              : _winName(winName), _rgbs( rgbs ), _inited(false), _outPath( outPath ), _zoomPoint(100,100), _zoomScale(1.)
                , _mapper( mapper ), _cuboidId(0), _frameIds( frameIds ), _vis(nullptr)
          {
              if ( tracks )
                  _tracks = *tracks;
          }

          void init();

          template <typename _TransformT>
          static void drawCuboid2( cv::Mat rgb, const _TransformT& T, const tracking::Mapper& mapper )
          {
              std::vector< Eigen::Vector2f > corners2D;
              bundle_physics::getCorners2D( corners2D, T, mapper );

              std::vector<cv::Scalar> colors = { {225,165,6}, {98,179,51}, {140,149,148}, {247,45,163}, {248,59,53}, {18,202,254}, {101,85,207}, {101,85,207}, {1,163,165} };
              //cv::Scalar edgeColor (255,30,30);

              auto const& edges = Cuboid::getEdges();
              for ( size_t i = 0; i < edges.size(); i+=2 )
              {
                  cv::line( rgb,
                      cv::Point2i( corners2D.at(edges[i])(0), corners2D.at(edges[i])(1) ),
                      cv::Point2i( corners2D.at(edges[i+1])(0), corners2D.at(edges[i+1])(1) ),
                      colors.at( i/2 % colors.size() ) );

              }
          }

          template <typename _TransformT>
          void drawCuboid( const _TransformT& T, cv::Mat& rgb, const tracking::Mapper& mapper )
          {
              std::vector<cv::Scalar> colors = { {225,165,6}, {98,179,51}, {140,149,148}, {247,45,163},
                                                 {18,202,254}, {248,59,53}, {101,85,207}, {1,163,165},
                                                 {139,69,19}, {30,144,255}, {238,232,170}, {75,0,130} };

              //auto T = cuboid.getTransform4x4(_frameId);
              std::vector< Eigen::Vector2f > corners2D;
              bundle_physics::getCorners2D( corners2D, T, mapper );

              std::vector< int > ids(8,-1);

              char text[255];
              int col(0);
              for ( auto const& corner2D : corners2D )
              {
                  float minDist = std::numeric_limits<float>::max();
                  int minId(-1);
                  int trackId(0);
                  for ( auto const& track : _tracks )
                  {
                      if ( track.hasPoint(_frameId) )
                      {
                          float diff = (corner2D - track.getPoint(_frameId)).norm();
                          if ( diff < minDist )
                          {
                              minDist = diff;
                              minId = trackId;
                          }
                      }
                      ++trackId;
                  }

                  if ( minId >= 0 )
                  {
                      auto const& trackPoint = _tracks.getTrack(minId).getPoint(_frameId);
                      cv::line( rgb, cv::Point2i(corner2D(0),corner2D(1)), cv::Point2i(trackPoint(0), trackPoint(1)), cv::Scalar(0.,0.,0.), 2 );
                      ids[col] = minId;
                  }

                  sprintf( text, "%d", col );
                  cv::putText( rgb, text, cv::Point2i( corners2D[col](0), corners2D[col](1) ), 1, 1, cv::Scalar(255,255,255) );
                  ++col;
              }

              cv::Scalar edgeColor (255,255,200);

              std::set<int> uniqueIds;
              uniqueIds.insert( ids.begin(), ids.end() );
              bool unique( false );
              if ( uniqueIds.size() == ids.size() )
              {
                  edgeColor = cv::Scalar( 20, 200, 20 );
                  std::cout << "unique" << std::endl;
                  if ( int(_cornerIds.size()) <= _cuboidId )
                  {
                      _cornerIds.resize( _cuboidId + 1 );
                  }
                  _cornerIds.at( _cuboidId ) = ids;
                  std::cout<<"_cornerIds[_cuboidId]:";for(size_t vi=0;vi!=_cornerIds[_cuboidId].size();++vi)std::cout<<_cornerIds[_cuboidId][vi]<<" ";std::cout << "\n";
                  unique  = true;
              }


              auto const& edges = Cuboid::getEdges();
              for ( size_t i = 0; i < edges.size(); i+=2 )
              {
                  cv::line( rgb,
                      cv::Point2i( corners2D.at(edges[i])(0), corners2D.at(edges[i])(1) ),
                      cv::Point2i( corners2D.at(edges[i+1])(0), corners2D.at(edges[i+1])(1) ),
                      unique ? edgeColor : colors.at( i/2 % colors.size()) );
              }

              Eigen::Vector3f center = T.translation();
              Eigen::Vector2f center2D = mapper.to2D( center, 0, false ).template head<2>();
              Eigen::Vector2f unitX2D = mapper.to2D( center + T.rotation() * Vector3::UnitX()/5., 0, false ).template head<2>();
              Eigen::Vector2f unitY2D = mapper.to2D( center + T.rotation() * Vector3::UnitY()/5., 0, false ).template head<2>();
              Eigen::Vector2f unitZ2D = mapper.to2D( center + T.rotation() * Vector3::UnitZ()/5., 0, false ).template head<2>();
              cv::line( rgb, cv::Point2i(center2D(0),center2D(1)), cv::Point2i(unitX2D(0),unitX2D(1)), cv::Scalar(0,0,255) );
              cv::line( rgb, cv::Point2i(center2D(0),center2D(1)), cv::Point2i(unitY2D(0),unitY2D(1)), cv::Scalar(0,255,0) );
              cv::line( rgb, cv::Point2i(center2D(0),center2D(1)), cv::Point2i(unitZ2D(0),unitZ2D(1)), cv::Scalar(255,0,0) );
          }

          void show();

          void upTrackId();

          void keyPressed( char c );

          void saveTracks( const std::string outPath );

          void addPoint( int x, int y );

          void callBackFunc( int event, int x, int y, int flags );

          void addCuboid( const Cuboid& cuboid );

          void show3D( const bool spin = false );

          std::string _winName;
          LinRgbsT       _rgbs;

      protected:
          FrameId                 _frameId;
          TrackId                 _trackId;
          bool                    _inited;
          tracking::Tracks2D      _tracks;
          std::vector<cv::Scalar> _colors;
          std::string             _outPath;
          cv::Point2i             _zoomPoint;
          float                   _zoomScale; // zoomScale
          tracking::Mapper        _mapper;
          std::vector< Cuboid >   _cuboids;
          CuboidId                _cuboidId;
          std::vector< std::vector<int> > _cornerIds; // for each cuboid: contains trackId for each of Cuboid::corner-s equivalents
          FrameIdsT               _frameIds;
          Soup::vis::Visualizer<Scalar>::PtrT _vis;
  }; //...cls Annotator

} //...ns tracking

#endif // TV_ANNOTATOR_H
