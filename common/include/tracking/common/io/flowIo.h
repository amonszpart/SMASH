// flowIO.h

#include <utility> // pair
#include <string>

namespace cv {
class Mat;
template<typename T> class Mat_;
template<typename T, int N> class Vec;
} //...ns cv

namespace io {
namespace flow {

// the "official" threshold - if the absolute value of either
// flow component is greater, it's considered unknown
//#define UNKNOWN_FLOW_THRESH 1e9
static constexpr float kUnknownFlowThresh = 1e9f;

// value to use to represent unknown flow
//#define UNKNOWN_FLOW 1e10
static constexpr float kUnknownFlow = 1e10f;

// return whether flow vector is unknown
bool unknown_flow(float u, float v);

bool unknown_flow(float* f);

/** \brief Read a flow file into 2-channel image */
cv::Mat_<cv::Vec<float,2> > readFlowFile(std::string const& filename);
std::pair<cv::Mat_<float>,cv::Mat_<float> > readFlowFileSplit(std::string const& filename);

/** \brief Writes a 2-channel image into a flow file. */
void writeFlowFile(cv::Mat_<cv::Vec<float, 2>> const& img, std::string const& filename);

/** \brief Writes two single channel images into a flow file. */
void writeFlowFile(cv::Mat_<float> const& flowX, cv::Mat_<float> const& flowY, std::string const& filename);

} //...ns flow
} //...ns io
