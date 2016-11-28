//
// Created by bontius on 05/02/16.
//
#include "tracking/common/io/depthIo.h"
#include "tracking/common/util/impl/parse.hpp"
#include "tracking/common/io/pgm.h"
#include "opencv2/core/core.hpp"
#include <iostream>

namespace tracking {

  /** \brief Reads all images in timespan. */
  int readDepths(tracking::DepthsT& depths, int argc, const char* argv[], const FrameIdsT& frameIds, float const scale) {
      std::string depthPattern("");
      if (!console::parse_arg(argc,argv,"--depth-pattern",depthPattern)) {
          return EXIT_FAILURE;
      }
      return readDepths(depths, depthPattern, frameIds, scale);
  }

  int readDepths(tracking::DepthsT& depths, std::string imgPattern, const FrameIdsT& frameIds, float const scale) {
      char depPath[2048];
      for (FrameId frameId = frameIds.front(); frameId <= frameIds.back(); step(frameId) ) { //inclusive!
          sprintf(depPath, imgPattern.c_str(), frameId);

          cv::Mat_<float> depth;
          if (!readPgm(depPath, depth, scale) || depth.empty()) {
              std::cerr << "can't read " << depPath << std::endl;
              return EXIT_FAILURE;
          }
          depths.insert(std::make_pair(frameId,depth));
      } //...for frames

      return EXIT_SUCCESS;
  } //...readImages()
}//...ns tracking