#ifndef AM_BROXIO_HPP
#define AM_BROXIO_HPP

#include "tracking/common/io/broxIO.h"
#include "tracking/common/io/sequentialReader.h"
#include "opencv2/core.hpp"
#include <memory>

namespace tracking {
    int BroxIO::readImage( const std::string& path, cv::Mat& imgOut )
    {
        ImageT img;
        img.readFromPPM( path.c_str() );
        cTensorToMat( img, imgOut );

        return 0;
    }

    int BroxIO::cTensorToMat( const ImageT& image, cv::Mat& out )
    {
        out = cv::Mat::zeros(image.ySize(), image.xSize(), CV_8UC3);
        //std::cout << "sizes: " << image.xSize() << "," << image.ySize() << "," << image.zSize() << std::endl;
        //std::cout << "step: " << out.step << "," << out.step1() << std::endl;
        //std::cout << "img.rows: " << out.rows << ", img.cols: " << out.cols << std::endl;
        for ( int y = 0; y != out.rows; ++y )
            for ( int x = 0; x != out.cols; ++x )
            {
                out.at<uchar>( y * out.step1() + x * out.channels() + 0 ) = image.operator()(x,y,2);
                out.at<uchar>( y * out.step1() + x * out.channels() + 1 ) = image.operator()(x,y,1);
                out.at<uchar>( y * out.step1() + x * out.channels() + 2 ) = image.operator()(x,y,0);
            }

        return 0;
    } //...cTensorToMat()

    int BroxIO::readSequence(const std::string &bmfPath, std::vector<ImagePtrT> &images, bool isCars)
    {
        images.clear();

        std::string s = bmfPath;
        s.erase( s.find_last_of("/")+1, s.length() );
        std::string inputDir = s;

        s = bmfPath;
        s.erase(0,s.find_last_of('.'));

        std::vector<std::string> imagePaths;

        // Read image sequence in bmf format
        if (s == ".bmf" || s == ".BMF")
        {
            int aImageCount,aViewCount;
            std::ifstream aStream( bmfPath.c_str() );
            aStream >> aImageCount;
            aStream >> aViewCount;
            for (int i = 0; i < aImageCount; i++)
            {
                std::string s;
                aStream >> s;
                imagePaths.push_back( inputDir + s );
            }
        }
        else
        {
            std::cout << "Must pass a bmf file as input" << std::endl;
            return -1;
        }

        for ( size_t t = 0; t < imagePaths.size(); ++t )
        {
            // Load next image
            images.push_back( ImagePtrT( new ImageT() ) );
            if ( isCars )
                images.back()->readFromPPMCars( imagePaths[t].c_str() );
            else
                images.back()->readFromPPM( imagePaths[t].c_str() );
            std::cout << "read " << imagePaths[t] << std::endl;
        }

        return images.size();
    } //...readSequence()

    // http://codeyarns.com/2015/09/08/how-to-compute-intrinsic-camera-matrix-for-a-camera/
    // 583.2829786373293 0.0               320.0
    // 0.0               579.4112549695428 240.0
    // 0.0               0.0               1.0
    int BroxIO::writeTracks( const Tracks2D& tracks, std::string ofPath )
    {
        std::ofstream fOut( ofPath, std::ios_base::binary | std::ios_base::trunc );
        SequentialWriter::writeField( &fOut, tracks.getSequenceLength() );
        //writeField( &fOut, tracks.getTrackCount() );
        //for ( auto const& track : tracks )
        for ( size_t i = 0; i != tracks.getTrackCount(); ++i )
        {
            auto const& track( tracks.at(i) );
            SequentialWriter::writeField( &fOut, track.getLabel() );
            SequentialWriter::writeField( &fOut, static_cast<size_t>(track.getPoints().size()) );
            //std::cout << "points.size: " << track.getPoints().size() << std::endl; fflush(stdout); fflush(stdout);
            for ( auto it = track.getPoints().begin(); it != track.getPoints().end(); ++it )
            //for ( auto const& pair : track.getPoints() )
            {
                fOut.write( reinterpret_cast<const char*>(it->second.getPoint().data()), sizeof(it->second.getPoint()) );
                //std::cout << "wrote " << it->second.p.transpose() << ", with " << sizeof(it->second.p) << std::endl; fflush(stdout);
                //fOut.write( reinterpret_cast<const char*>(pair.second.p.data()), sizeof(pair.second.p) );
                SequentialWriter::writeField( &fOut, it->first );
                //writeField( &fOut, pair.second.frame );
                //std::cout << "wrote frame " << it->second.frame << std::endl; fflush(stdout);
            }
            //std::cout << "finished " << i << std::endl; fflush(stdout);
        }
        fOut.close();
        std::cout << "[" << __func__ << "]: " << "wrote to " << ofPath << std::endl; fflush(stdout);

        return EXIT_SUCCESS;
    }

    int BroxIO::readTracks(std::string path, Tracks2D& tracks, const bool relabel) {
        if ( path.find(".dat") != std::string::npos ) {
            std::cout << "[" << __func__ << "]: " << "reading ascii" << std::endl;
            BroxIO::readTracksAscii( path, tracks, relabel );
        } else {
            std::cout << "[" << __func__ << "]: " << "reading binary" << std::endl;
            BroxIO::readTracksBinary( path, tracks, relabel );
        }

        return tracks.size();
    } //...readTracks()

    int BroxIO::readTracksAscii( std::string path, Tracks2D& tracks, const bool relabel ) {
        int sequenceLength(0);
        std::ifstream aFile( path.c_str() );
        // Read number of frames considered
        aFile >> sequenceLength;
        tracks.setSequenceLength( sequenceLength );
        // Read number of tracks
        Tracks2D::CountT aCount;
        aFile >> aCount;
        std::cout << "[" << __func__ << "]: " << "reading " << aCount << " tracks " << std::endl;
        // Read each track
        for (int i = 0; i != static_cast<int>(aCount); ++i) {
            // Read label and length of track
            int aSize;
            Track2D& aTrack = tracks.addTrack(Track2D{i});
            aFile >> aTrack.getLabel();
            if (relabel)
                aTrack.setLabel(i);
            aFile >> aSize;
            // Read x,y coordinates and frame number of the tracked point
            TrackPoint2D point;
            FrameId frame;
            for (int j = 0; j != aSize; ++j) {
                aFile >> point.getPoint()(0);
                aFile >> point.getPoint()(1);
                aFile >> frame;
                aTrack.addPoint(frame, point);
            }
        }
        std::cout << "read " << tracks.getTrackCount() << " tracks for sequence " << path << std::endl;
        return tracks.size();
    } //...readTracks()

    /**
     * @brief BroxIO::readTracksBinary
     * @param path
     * @param[out] tracks
     * @param[in] relabel Set track labels to sequential order in output to create unique IDs for this run.
     * @return
     */
    int BroxIO::readTracksBinary(std::string path, Tracks2D& tracks, bool relabel) {
        std::ifstream f(path.c_str(), std::ios::in | std::ios_base::binary);
        if (!f.is_open()) {
            std::cerr << "[" << __func__ << "]: " << "could not open " << path << std::endl;
            return 1;
        }
        tracks.clear();

        Tracks2D::CountT sequenceLength(0);
        SequentialReader::readField( &f, sequenceLength );
        tracks.setSequenceLength( sequenceLength );

        std::vector<char> data;
        while (true) {
            Tracks2D::CountT trackLength( 0 );
            Track2D::LabelT label( 0 );
            SequentialReader::readField( &f, label );
            // quit, if no more valid data in stream
            if (!f.good()) {
                std::cout << "[" << __func__ << "] no f.good false" << std::endl;
                break;
            }
            if ( relabel )
                label = tracks.getTrackCount();
            Track2D& track = tracks.addTrack(Track2D{0});
            track.setLabel(label);
            SequentialReader::readField( &f, trackLength );
            if ( data.size() < trackLength * TrackPoint2D::SizeInBinary() )
                data.resize( trackLength * TrackPoint2D::SizeInBinary() );
            f.read( data.data(), trackLength * TrackPoint2D::SizeInBinary() );

            TrackPoint2D point;
            size_t offset( 0 );
            FrameId frame;
            for ( FrameId j = 0; j != trackLength; ++j )
            {
                //f.read( reinterpret_cast<char*>(point.p.data()), 2 * sizeof(*point.p.data()) );
                //readField( &f, point.frame );
                //point.p = Eigen::Map< const TrackPoint::Point2 >( reinterpret_cast<TrackPoint::Point2::Scalar*>(&(data[offset])) );
                point.getPoint()(0) = *reinterpret_cast<float*>(&data[offset]);
                point.getPoint()(1) = *reinterpret_cast<float*>(&data[offset+sizeof(TrackPoint2D::Scalar)]);
                offset += sizeof(point.getPoint());

                frame = *reinterpret_cast< const FrameId* >( &(data[offset]) );
                offset += sizeof( frame );
                track.addPoint( frame, point );
            }
        }

        return tracks.getTrackCount();
    }

    /** \brief Converts ascii to binary without storing the full input in memory */
    int BroxIO::convertDatToBin(std::string ifPath, std::string ofPath, const int frameIdOffset) {
        std::cout << "[" << __func__ << "]: " << ifPath << " -> " << ofPath << std::endl;

        SequentialReader reader( ifPath, SequentialReader::ASCII );
        if ( !reader.init() )
        {
            std::cerr << "[" << __func__ << "]: " << "could not open " << ifPath << " for reading" << std::endl;
            return 1;
        }

        SequentialWriter writer( ofPath, reader.getSequenceLength(), SequentialReader::BINARY );
        if ( !writer.init() )
        {
            std::cerr << "[" << __func__ << "]: " << "could not open " << ofPath << " for writing" << std::endl;
            return 1;
        }

        // Read each track
        if ( frameIdOffset )
        {
            Track2D track, track2;
            while ( reader.readNext(track) )
            {
                track2.clear();
                for ( const auto& frameIdAndTrackPoint : track.getPoints() )
                {
                   track2.addPoint( frameIdAndTrackPoint.first + frameIdOffset, frameIdAndTrackPoint.second );
                }
                writer.writeNext( track2 );
            }
        }
        else
        {
            Track2D track;
            while ( reader.readNext(track) )
                writer.writeNext( track );
        }

        std::cout << "[" << __func__ << "]: " << "wrote to " << ofPath << std::endl; fflush(stdout);

        return 0;
    } //...convertDatToBin()

} //...ns tracking

#endif // AM_BROXIO_HPP
