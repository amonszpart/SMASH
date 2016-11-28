#ifndef AM_BROXIO_H
#define AM_BROXIO_H

#include "CTensor.h"
#include "tracking/common/trackFwDecl.h"
#include "tracking/common/tracksFwDecl.h"
#include <memory>

namespace cv { class Mat; }

namespace tracking {

  class BroxIO {
      public:
          typedef CTensor<float>          ImageT;
          typedef std::shared_ptr<ImageT> ImagePtrT;

          /** \brief Read a list of PPm-s to CTensor-s.
           * \param[in]  bmfPath List of input ppm-s.
           * \param[out] images
           * \param[in]  isCars  False, if it's ppm-s from "capture", or ppm-s from the original example ("capture" puts a ' ' after \n).
           * \return
           */
          static int readSequence( const std::string& bmfPath, std::vector< ImagePtrT >& images, bool isCars = false );
          static int cTensorToMat( const ImageT& image, cv::Mat& out );
          static int readImage( const std::string& path, cv::Mat& imgOut );
          static int readTracksAscii( std::string path, Tracks2D& tracks, const bool relabel = false  );
          //static int readTracksBinary( std::string path, Tracks& tracks );
          static int readTracksBinary( std::string path, Tracks2D& tracks, const bool relabel = false  );
          static int readTracks( std::string path, Tracks2D& tracks, const bool relabel = false );
          static int writeTracks( const Tracks2D& tracks, std::string ofPath );
          static int convertDatToBin( std::string ifPath, std::string ofPath, const int frameIdOffset = 0);
  }; //...BroxIO

} //...ns tracking

#endif // AM_BROXIO_H
