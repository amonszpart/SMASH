// flow_io.cpp
//
// read and write our simple .flo flow file format

// ".flo" file format used for optical flow evaluation
//
// Stores 2-band float image for horizontal (u) and vertical (v) flow components.
// Floats are stored in little-endian order.
// A flow value is considered "unknown" if either |u| or |v| is greater than 1e9.
//
//  bytes  contents
//
//  0-3     tag: "PIEH" in ASCII, which in little endian happens to be the float 202021.25
//          (just a sanity check that floats are represented correctly)
//  4-7     width as an integer
//  8-11    height as an integer
//  12-end  data (width*height*2*4 bytes total)
//          the float values for u and v, interleaved, in row order, i.e.,
//          u[row0,col0], v[row0,col0], u[row0,col1], v[row0,col1], ...
//


// first four bytes, should be the same in little endian
//#define TAG_FLOAT 202021.25  // check for this when READING the file
static constexpr float tag_float = 202021.25f;
//#define TAG_STRING "PIEH"    // use this when WRITING the file
static constexpr const char* tag_string = "PIEH";

#include "tracking/common/io/flowIo.h"
#include "opencv2/core.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <cstring>
#include <stdexcept>
#include <iostream>

namespace io {
namespace flow {

// return whether flow vector is unknown
bool unknown_flow(float u, float v) {
    return (std::fabs(u) > kUnknownFlowThresh)
           || (std::fabs(v) > kUnknownFlowThresh)
           || std::isnan(u) || std::isnan(v);
}

bool unknown_flow(float* f)
{ return unknown_flow(f[0], f[1]); }

std::pair<cv::Mat_<float>,cv::Mat_<float> > readFlowFileSplit(std::string const& filename) {
    cv::Mat_<cv::Vec2f> flow2 = readFlowFile(filename);
    std::vector< cv::Mat_<float> > out;
    cv::split(flow2, out);
    return {std::move(out[0]), std::move(out[1])};
}

cv::Mat_<cv::Vec2f> readFlowFile(std::string const& filename) {
    if (filename.empty()) {
        throw std::logic_error("ReadFlowFile: empty filename");
    }

    const char* dot = strrchr(filename.c_str(), '.');
    if (strcmp(dot, ".flo") != 0) {
        throw std::logic_error("ReadFlowFile (" + std::string(filename) + "): extension .flo expected");
    }

    FILE* stream = fopen(filename.c_str(), "rb");
    if (stream == 0) {
        throw std::logic_error("ReadFlowFile: could not open " + std::string(filename));
    }

    int   width, height;
    float tag;

    if ((int) fread(&tag, sizeof(float), 1, stream) != 1 ||
        (int) fread(&width, sizeof(int), 1, stream) != 1 ||
        (int) fread(&height, sizeof(int), 1, stream) != 1) {
            throw std::logic_error("ReadFlowFile: problem reading file " + std::string(filename));
    }

    if (tag != tag_float) { // simple test for correct endian-ness
        throw std::logic_error(
            "ReadFlowFile(" + std::string(filename) + "): wrong tag (possibly due to big-endian machine?)");
    }

    // another sanity check to see that integers were read correctly (99999 should do the trick...)
    if (width < 1 || width > 99999) {
        throw std::logic_error("ReadFlowFile(" + std::string(filename) + "): illegal width " + std::to_string(width));
    }

    if (height < 1 || height > 99999) {
        throw std::logic_error("ReadFlowFile(" + std::string(filename) + "): illegal height " + std::to_string(height));
    }

    cv::Mat   flow(height, width, CV_32FC2);
    //printf("reading %d x %d x 2 = %d floats\n", width, height, width*height*2);
    int const n  = 2 * width;
    for (int  y  = 0; y != height; ++y) {
        if ((int) fread(flow.row(y).ptr<float>(0), sizeof(float), n, stream) != n) {
            throw std::logic_error("ReadFlowFile(" + std::string(filename) + "): file is too short");
        }
    }

    if (fgetc(stream) != EOF) {
        throw std::logic_error("ReadFlowFile(" + std::string(filename) + "): file is too long");
    }

    fclose(stream);
    return flow;
} //...ReadFlowFile()


/** \brief               Writes a 2-band image into flow file
 *  \param[in] img       Image with type CV_32FC1.
 *  \param[in] filename  Filename.
 */
void writeFlowFile(cv::Mat_<cv::Vec2f> const& img, std::string const& filename) {
    if (filename.empty()) {
        throw std::logic_error("WriteFlowFile: empty filename");
    }

    const char* dot = strrchr(filename.c_str(), '.');
    if (dot == NULL) {
        throw std::logic_error("WriteFlowFile: extension required in filename '" + std::string(filename) + "'");
    }

    if (strcmp(dot, ".flo") != 0) {
        throw std::logic_error("WriteFlowFile: filename '" + std::string(filename) + "' should have extension '.flo'");
    }

    int const width  = img.cols,
              height = img.rows,
              nBands = img.channels();

    if (nBands != 2) {
        throw std::logic_error(
            "WriteFlowFile(" + std::string(filename) + "): image must have 2 bands, has " + std::to_string(nBands));
    }


    FILE* stream = fopen(filename.c_str(), "wb");
    if (stream == 0) {
        throw std::logic_error("WriteFlowFile: could not open " + std::string(filename));
    }

    // write the header
    fprintf(stream, tag_string);
    if ((int) fwrite(&width, sizeof(int), 1, stream) != 1 ||
        (int) fwrite(&height, sizeof(int), 1, stream) != 1) {
            throw std::logic_error("WriteFlowFile(" + std::string(filename) + "): problem writing header");
    }

    // write the rows
    int      n = nBands * width;
    for (int y = 0; y < height; y++) {
        if ((int) fwrite(img.row(y).ptr<float>(0), sizeof(float), n, stream) != n) {
            throw std::logic_error("WriteFlowFile(" + std::string(filename) + "): problem writing data");
        }
    }

    fclose(stream);
} //...WriteFlowFile()

void writeFlowFile(cv::Mat_<float> const& flowX, cv::Mat_<float> const& flowY, std::string const& filename) {
    if (flowX.channels() != 1 || flowY.channels() != 1) {
        throw new std::logic_error("Input images assumed to be CV_32FC1");
    }
    cv::Mat_<cv::Vec2f> tmp;
    cv::merge(std::vector<cv::Mat_<float> >{flowX, flowY}, tmp);
    writeFlowFile(tmp, filename);
} //...WriteFlowFile()

} //...ns io
} //...ns flow


/*
int main() {

    try {
	CShape sh(5, 1, 2);
	CFloatImage img(sh);
	img.ClearPixels();
	img.Pixel(0, 0, 0) = -5.0f;
	char *filename = "test.flo";

	WriteFlowFile(img, filename);
	ReadFlowFile(img, filename);
    }
    catch (CError &err) {
	fprintf(stderr, err.message);
	fprintf(stderr, "\n");
	exit(1);
    }

    return 0;
}
*/
