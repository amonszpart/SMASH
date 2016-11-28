#include "tracking/common/io/sequentialReader.h"
#include "tracking/common/trackPoint3d.h"

namespace tracking {
    bool SequentialReader::init()
    {
        if ( _stream != nullptr )
            std::cerr << "[" << __func__ << "]: " << "already opened..." << std::endl;

        if ( _type == BINARY )
            _stream.reset( new std::ifstream(_path, std::ios::in | std::ios_base::binary) );
        else if ( _type == ASCII )
            _stream.reset( new std::ifstream(_path, std::ios::in) );
        else
            throw new SequentialReader_UnknownTypeException("Unknown input file type. It should be ASCII/BINARY");

        if ( !_stream->is_open() )
        {
            std::cerr << "could not open " << _path << std::endl;
            return false;
        }

        if ( _type == ASCII )
        {
            // Read number of frames considered
            *_stream >> _sequenceLength;

            // Read number of tracks
            Tracks2D::CountT trackCount;
            *_stream >> trackCount;
        }
        else if ( _type == BINARY )
        {
            // Read number of frames considered
            readField( _stream.get(), _sequenceLength );
            // Read number of tracks
            //readField( _stream.get(), _trackCount );
        }

        return true;
    } //...init(0

    bool SequentialReader::readNext( Track2D& track )
    {
        if ( _type == ASCII )
            return readNextAscii( track );
        else if ( _type == BINARY )
            return readNextBinary( track );
        else
            throw new SequentialReader_UnknownTypeException("Unknown input file type. It should be ASCII/BINARY");
    }

    bool SequentialReader::readNextAscii( Track2D& track )
    {
        if ( !_stream || !_stream->is_open() )
            this->init();

        track.clear();

        Tracks2D::CountT      trackLength;
        TrackPoint2D          point;
        FrameId frame;

        // Read label and length of track
        *_stream >> track.getLabel();
        if ( !_stream->good() )
            return false;
        *_stream >> trackLength;
        for ( Tracks2D::CountT j = 0; j < trackLength; ++j )
        {
            *_stream >> point.getPoint()(0) >> point.getPoint()(1);
            *_stream >> frame;
            track.addPoint( frame, point );
        }

        return true;
    } //...readNextAscii()

    bool SequentialReader::readNextBinary( Track2D& track )
    {
        if ( !_stream || !_stream->is_open() )
            this->init();

        track.clear();

        //for ( size_t i = 0; i != trackCount; ++i )
        //{
        //tracks.emplace_back( Track() );
        //Track& track( tracks.back() );
        readField( _stream.get(), track.getLabel() );
        if ( !_stream->good() )
            return false;
        Tracks2D::CountT trackLength( 0 );
        readField( _stream.get(), trackLength );
        if ( _buffer.size() < trackLength * TrackPoint2D::SizeInBinary() )
            _buffer.resize( trackLength * TrackPoint2D::SizeInBinary() );
        _stream->read( _buffer.data(), trackLength * TrackPoint2D::SizeInBinary() );

        TrackPoint2D point;
        size_t offset( 0 );
        FrameId frame;
        for ( FrameId j = 0; j != trackLength; ++j )
        {
            point.getPoint()(0) = *reinterpret_cast<float*>(&_buffer[offset]);
            point.getPoint()(1) = *reinterpret_cast<float*>(&_buffer[offset+sizeof(TrackPoint2D::Scalar)]);
//            std::cout << "offset: " << sizeof(point.getPoint()) << std::endl;
            offset += sizeof(point.getPoint());

            frame = *reinterpret_cast< const FrameId* >( &(_buffer[offset]) );
            offset += sizeof( frame );
            track.addPoint( frame, point );
        }

        return true;
    } //...readNextBinary()

} //...ns tracking()
