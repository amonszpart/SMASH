#ifndef TRACKVIDEO_COMMON_SEQUENTIALREADER_H
#define TRACKVIDEO_COMMON_SEQUENTIALREADER_H

#include "tracking/common/trackPoint2d.h"
#include "tracking/common/track.h"
#include "tracking/common/tracks.h"
#include "tracking/common/util/exception.h"
#include <memory>
#include <fstream>
#include <locale>
#include <string>

namespace tracking {
    class SequentialIO {
        public:
            enum Type { ASCII, BINARY };
            SequentialIO( const std::string& path, const Type type )
                : _type(type), _path( path ), _sequenceLength(0) { }
            SequentialIO( const std::string& path, const Tracks2D::CountT sequenceLength, const Type type )
                : _type(type), _path( path ), _sequenceLength(sequenceLength) { }

            inline Tracks2D::CountT getSequenceLength() const { return _sequenceLength; }
        protected:
            Type            _type;
            std::string     _path;
            Tracks2D::CountT  _sequenceLength;
    }; //...class SequentialIO

    class SequentialReader : public SequentialIO {
        public:
            DEFINE_EXCEPTION(SequentialReader_CouldNotOpenInputFile);
            DEFINE_EXCEPTION(SequentialReader_UnknownType);

            SequentialReader( const std::string& path, const Type type )
                :  SequentialIO( path, type ), _stream(nullptr) {}

            bool init();
            bool readNext( Track2D& track );

            bool readNextAscii( Track2D& track );
            bool readNextBinary( Track2D& track );

            template <typename T>
            static inline void readField(std::ifstream* f, T& field) {
                f->read(reinterpret_cast<char*>(&field), sizeof(field));
            }
        protected:
            std::unique_ptr<std::ifstream> _stream;

            using SequentialIO::_type;
            using SequentialIO::_path;
            using SequentialIO::_sequenceLength;

            std::vector<char>              _buffer; // used by binary
    }; //...SequentialReader

    class SequentialWriter : public SequentialIO {
        public:
            DEFINE_EXCEPTION(SequentialIO_ASCIIUnimplemented)
            SequentialWriter( const std::string& path, Type type ) = delete;
            SequentialWriter( const std::string& path, Tracks2D::CountT sequenceLength, Type type = BINARY )
                : SequentialIO( path, sequenceLength, type ), _stream(nullptr)
            {
                if ( _type == ASCII )
                {
                    throw new SequentialIO_ASCIIUnimplementedException("TODO");
                }
            }

            bool init()
            {
                _stream.reset( new std::ofstream(_path, std::ios_base::binary | std::ios_base::trunc) );

                // Read number of frames considered
                writeField( _stream.get(), _sequenceLength );

                return _stream->is_open();
            }

            bool writeNext( const Track2D& track )
            {
                if ( !_stream || !_stream->is_open() )
                    this->init();

                writeField( _stream.get(), track.getLabel() );
                writeField( _stream.get(), track.getPoints().size() );

                for ( auto const& frameIdAndTrackPoint : track.getPoints() )
                {
//                    std::cout << "point: " << frameIdAndTrackPoint.second.getPoint().transpose() << std::endl;
//                    std::cout << "writing: "
//                              << *reinterpret_cast<const float*>(reinterpret_cast<const char*>(frameIdAndTrackPoint.second.getPoint().data()))
//                              << ","
//                              << *reinterpret_cast<const float*>(reinterpret_cast<const char*>(frameIdAndTrackPoint.second.getPoint().data())+sizeof(frameIdAndTrackPoint.second.getPoint())/2)
//                              << " (offset: " << sizeof(frameIdAndTrackPoint.second.getPoint())/2 << ")"
//                              << std::endl;
                    //std::cout << "sizeof:" << sizeof(frameIdAndTrackPoint.second.getPoint()) << " vs. sizeinBainary:" << frameIdAndTrackPoint.second.SizeInBinary() << std::endl;
                    _stream->write( reinterpret_cast<const char*>(frameIdAndTrackPoint.second.getPoint().data()), sizeof(frameIdAndTrackPoint.second.getPoint()) );
                    writeField( _stream.get(), frameIdAndTrackPoint.first );
                }
                return true;
            } //...writeNext()

            template <typename T>
            static inline void writeField( std::ofstream* const stream, const T value )
            {
                //std::cout << "writing " << value << " over " << sizeof(value) << " bytes " << std::endl;
                stream->write( reinterpret_cast<const char*>(&value), sizeof(value) );
            }

        protected:
            std::unique_ptr<std::ofstream> _stream;
            using SequentialIO::_type;
            using SequentialIO::_path;
            using SequentialIO::_sequenceLength;
    }; //...SequentialWriter
} //...ns tracking

#endif // TRACKVIDEO_COMMON_SEQUENTIALREADER_H
