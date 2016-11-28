#ifndef TRACKVIDEO_COMMON_OPTIONSPARSER_H
#define TRACKVIDEO_COMMON_OPTIONSPARSER_H

#include "tracking/common/groupedTracksFwDecl.h"
#include "tracking/common/typedefs.h"

namespace tracking {
class Mapper;

int parseMapper(Mapper& mapper, int argc, const char** argv);

int parseFrameIds(FrameIdsT& frameIds, int argc, const char** argv, std::string const& flag = "--frames",
                  bool required = true);

int parseImgPaths(std::vector<std::string>& imgPaths, int argc, const char** argv, const FrameIdsT& frameIds,
                  bool allowDefault = false, std::string* outPattern = nullptr);

int parseDepthPaths(std::vector<std::string>& depthPaths, int argc, const char** argv, const FrameIdsT& frameIds,
                    bool allowDefault = false);

int parseGroupedTracks(GroupedTracks2d& groupedTracks2d, int argc, const char** argv, std::string* datPathArg = nullptr,
                       TrackId* const maxTrackIdArg = nullptr);

} //...ns tracking

#endif // TRACKVIDEO_COMMON_OPTIONSPARSER_H
