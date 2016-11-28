#include "tracking/common/io/rgbIo.h"
#include "tracking/common/io/impl/rgbIo.hpp"
#include "tracking/common/util/impl/parse.hpp" //console
#include "tracking/common/util/exception.h"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

namespace tracking {
/** \brief Reads all images in timespan. */
int readImages( tracking::LinRgbsT& rgbs, int argc, const char* argv[], const FrameIdsT& frameIds) {
    std::string imgPattern("");
    if (!console::parse_arg(argc,argv,"--img-pattern",imgPattern)) {
        std::cerr << "[" << __func__ << "]: " << "you should specify the input image pattern by --img-pattern" << std::endl;
        return EXIT_FAILURE;
    }
    return readImages( rgbs, imgPattern, frameIds );
}

/** \brief Reads all images in timespan. */
int readImages(tracking::LinRgbsT& rgbs, std::string imgPattern, const FrameIdsT& frameIds) {
    char imgPath[2048];
    for (FrameId frameId = frameIds.front(); frameId <= frameIds.back(); ++frameId) { //inclusive!
        sprintf(imgPath, imgPattern.c_str(), frameId);
        //std::cout << "reading " << imgPath << std::endl;
        cv::Mat tmp = cv::imread(imgPath);
        if (tmp.empty()) {
            std::cerr << "[" << __func__ << "]: " << imgPath << " failed" << std::endl;
            return EXIT_FAILURE;
        } else {
            rgbs.emplace_back(tmp);
        }
    } //...for frames

    return EXIT_SUCCESS;
} //...readImages()

DEFINE_EXCEPTION(WriteRgb_ThreeChannelsExpected)
DEFINE_EXCEPTION(WriteRgb_ImgNotContinuous)
void writeRgb(cv::Mat const& img, const std::string path) {
    if (img.channels() != 3)
        throw new WriteRgb_ThreeChannelsExpectedException(std::to_string(img.channels()));
    if (!img.isContinuous())
        throw new WriteRgb_ImgNotContinuousException("");
    cv::Mat dst;
    cv::cvtColor( img, dst, cv::COLOR_BGR2RGB );
    FILE *fp;
    fp = fopen(path.c_str(), "wb");
    if ( !fp ) {
        std::cerr << "[" << __func__ << "]: " << "could not open " << path << std::endl;
        return;
    }
    fprintf(fp, "P6 %d %d 255\n", img.cols, img.rows );
    fwrite( dst.data, img.cols * img.rows * 3 * sizeof(uchar), 1, fp );
    fclose(fp);
    std::cout << "[" << __func__ << "]: " << "wrote to " << path << std::endl;
}
} //...ns tracking
