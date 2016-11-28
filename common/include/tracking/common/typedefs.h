#ifndef TRACKVIDEO_COMMON_TYPEDEFS
#define TRACKVIDEO_COMMON_TYPEDEFS

#include <vector>
#include <map>
#include <set>

namespace cv { class Mat; template <typename T> class Mat_; }

namespace tracking {
/** \defgroup common
 * Module to work with 2D and 3D tracks from:
 * Keuper, Andres and Brox:  Motion Trajectory Segmentation via Minimum Cost Multicuts
 * http://lmb.informatik.uni-freiburg.de/Publications/2015/KB15b/motionSeg.pdf
 *  @{
 */

/** \brief Discrete measure of time. Denotes the time of the observation (recording) made. */
typedef unsigned FrameId;

/** \brief Global function for looking forward in time. */
inline FrameId getNext ( const FrameId  frameId ) { return frameId+1; }
/** \brief Global function for looking backward in time. */
inline FrameId getPrev ( const FrameId  frameId ) { return frameId-1; }
/** \brief Global function for stepping forward in time. */
inline FrameId step    (       FrameId& frameId ) { return ++frameId; }
/** \brief Global function for stepping backward in time. */
inline FrameId backStep(       FrameId& frameId ) { return --frameId; }

/** \brief Container for a list of \ref FrameId objects to consider. */
typedef std::vector<FrameId>    FrameIdsT;
typedef std::set<FrameId>    AllFrameIdsT;

inline FrameId getFirst(AllFrameIdsT const& allFrameIds) { return *allFrameIds.begin(); }
inline FrameId getLast(AllFrameIdsT const& allFrameIds ) { return *allFrameIds.rbegin(); }

/** \brief Unique id for a track (2D or 3D point). */
typedef unsigned                TrackId;

/** \brief Linear id type. An alias for \ref TrackId, but explicitly denotes linear types. */
typedef TrackId                 LinId;
/** \brief Unique id for a group of tracks. */
typedef TrackId                 GroupId;

/** \brief Container for input colour images. */
typedef std::vector<cv::Mat        > LinRgbsT;
/** \brief Indexed container for input colour images. */
typedef std::map<FrameId,cv::Mat   > RgbsT;

/** \brief Depth stored as meters in single precision floating point: CV_32FC1. */
typedef cv::Mat_<float> DepthT;
/** \brief Container for input depth images, indexed by time. */
typedef std::map   <FrameId, DepthT> DepthsT;

/** \brief Constant for not known or initialized \ref TrackId. */
static constexpr TrackId INVALID_TRACK_ID = TrackId(-1);
/** \brief Constant for not specified \ref FrameId. */
static constexpr FrameId INVALID_FRAME_ID = FrameId(-1);

/** @} (common) */

} //...tracking

#endif // TRACKVIDEO_COMMON_TYPEDEFS

#if 0
//    struct FrameId
//    {
//        public:
//            FrameId() : _frameId(0u), _step(1u) {}
//            FrameId( const unsigned frameId, const int step = 1) : _frameId(frameId), _step(step) {}
//            operator unsigned() const { return _frameId; }
//            FrameId& operator++() //prefix
//            {
//                _frameId += _step;
//                return *this;
//            }
//            FrameId& advance( int superStep ) { _frameId += _step * superStep; return *this; }
//            FrameId operator++(int v) = delete; //postfix
//            //FrameId& step() { _frameId += _step; return *this; }
//            FrameId getNext() const { return _frameId + _step; }
//            friend std::istream& operator>>(std::istream& is, FrameId& frameId);
//        protected:
//            unsigned _frameId;
//            int      _step; //!< the next considered frame has this value
//    }; //...FrameId
//    inline std::istream& operator>>(std::istream& is, FrameId& frameId)
//    {
//        return is >> frameId._frameId;
//    }
#endif