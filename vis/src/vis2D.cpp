#include "tracking/common/trackPoint2d.h"
#include "tracking/common/trackPoint3d.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/util/colors.h"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/core.hpp"
#include <string>
#include <list>

namespace tracking {
    int showTracks2D(const std::string& imgPattern, const tracking::Tracks2D& tracks, const FrameId startFrame, const FrameId endFrame, const int nTracks ) {
        //const int from(0), step(10), end(460), nTracks(3000);
        std::string dest( "mov/masked_%05d.jpg" );

        std::vector<size_t> trackIndices;
        const float ratio = nTracks ? nTracks / static_cast<float>(tracks.getTrackCount()) : 1.;
        for ( size_t trackId = 0; trackId != tracks.getTrackCount(); ++trackId )
        {
            if ( rand() / static_cast<float>(RAND_MAX) < ratio )
                trackIndices.push_back( trackId );
        }
        std::cout << "[" << __func__ << "]: " << "showing " << trackIndices.size() << " tracks" << std::endl;
        //std::vector<cv::Vec3b> colours = nColoursCv<cv::Vec3b>( trackIndices.size(), 255., true );
        std::vector<cv::Scalar> colours = colors::nColoursCv<cv::Scalar>( trackIndices.size(), 255., true );

        std::map< TrackId, std::list<cv::Point> > history;
        char pointName[255];
        cv::Mat img;
        char imgPath[1024], outPath[1024];
        int delay = 0;
        for ( FrameId frameId = startFrame; frameId <= endFrame; frameId += 1 )
        {
            sprintf( imgPath, imgPattern.c_str(), frameId );
            img = cv::imread( imgPath, cv::IMREAD_COLOR );

            int indexId(0);
            for ( auto const& index : trackIndices )
            {
                const Track2D& track = tracks.getTrack(index);

                if ( track.hasPoint(frameId) )
                {
                    const TrackPoint2D& trackPoint = track.getPoint( frameId );
                    if ( trackPoint.getPoint()(1) < img.rows
                         && trackPoint.getPoint()(0) < img.cols
                         && trackPoint.getPoint()(1) >= 0
                         && trackPoint.getPoint()(0) >= 0 )
                    {
                        cv::Point cvPoint( trackPoint.getPoint()(0), trackPoint.getPoint()(1));
                        cv::circle( img, cvPoint, 2, colours.at(indexId), -1 );
                        sprintf( pointName, "%d", track.getLabel() );
                        cv::putText( img, pointName, cvPoint, 1, 1, colours.at(indexId) );
                        //cv::line( img, cv::Point(0,0), cvPoint, colours.at(indexId), 3 );
                        history[index].push_back(cvPoint);
                    }
//                    else
//                        std::cout << "skipping " << trackPoint.getPoint() << std::endl;
                    auto it2 = history[index].begin();
                    ++it2;
                    for ( auto it = history[index].begin(); it2 != history[index].end(); ++it, ++it2 )
                    {
                        cv::line( img, *it, *it2, colours.at(indexId), 1 );
                    }
                }
                //else
                    //std::cout << "track " << index << " has no point in frameId " << frameId << std::endl;

                ++indexId;
            }

            sprintf( pointName, "%u", frameId );
            cv::putText( img, pointName, cv::Point(10,20), 1, 1, cv::Scalar(255,255,255) );

            cv::imshow( "img", img );

            sprintf( outPath, dest.c_str(), static_cast<int>(frameId) );
            cv::imwrite( outPath, img );
            std::cout << "wrote " << outPath << std::endl;
            char c = 0;

                c = cv::waitKey( delay );
                if ( c == 32 )
                    if ( delay )
                        delay = 0;
                    else
                        delay = 150;
                else if ( c == 13 )
                    delay = 150;
            //} while ( c != 32 );
        }

        return 0;
    } //... showTracks
} //...ns tracking
