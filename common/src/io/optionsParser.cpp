#include "tracking/common/io/optionsParser.h"

#include "tracking/common/io/segmentedTracksIo.h"
#include "tracking/common/groupedTracks.h"
#include "tracking/common/mapper.h"
#include "tracking/common/typedefs.h"
#include "tracking/common/util/impl/parse.hpp" //console
#include "tracking/common/macros.inl"
#include <fstream>

namespace tracking {

int parseMapper(Mapper& mapper, int argc, const char** argv) {
    int ret = EXIT_SUCCESS;
    Mapper::INTRINSICS intrinsicsName( Mapper::INTRINSICS::KINECT_ARON );
    std::string intrinsicsString;
    if ( console::parse_arg( argc,argv,"--intr",intrinsicsString ) )
    {
        if ( intrinsicsString.find(".") != std::string::npos )
        {
            std::ifstream fin( intrinsicsString );
            if ( !fin.is_open() )
            {
                std::cerr << "Could not open " << intrinsicsString << std::endl;
                return EXIT_FAILURE;
            }
            Mapper::Matrix3 intr( Mapper::Matrix3::Identity() );
            for (int row = 0; row != 3; ++row)
                for (int col = 0; col != 3; ++col)
                    fin >> intr(row, col);
            mapper = Mapper( intr );
            std::cout << "parsed intrinsics:\n" << mapper.getIntrinsics() << std::endl;
            return EXIT_SUCCESS;
        }
        else if ( intrinsicsString.compare("kinect-aron") == 0 )
            intrinsicsName = Mapper::INTRINSICS::KINECT_ARON;
        else if ( intrinsicsString.compare("adobe") == 0 )
            intrinsicsName = Mapper::INTRINSICS::PRIMESENSE;
        else if ( intrinsicsString.compare("blensor") == 0 )
            intrinsicsName = Mapper::INTRINSICS::BLENSOR;
        else if ( intrinsicsString.compare("dslr-niloy") == 0 )
            intrinsicsName = Mapper::INTRINSICS::DSLR_NILOY;
        else if ( intrinsicsString.compare("dslr-niloy2") == 0 )
            intrinsicsName = Mapper::INTRINSICS::DSLR_NILOY2;
        else if ( intrinsicsString.compare("iphone6") == 0 )
            intrinsicsName = Mapper::IPHONE6;
        else if ( intrinsicsString.compare("iphone6-half") == 0 )
            intrinsicsName = Mapper::IPHONE6HALF;
        else if ( intrinsicsString.compare("iphone6-half-lying") == 0 )
            intrinsicsName = Mapper::IPHONE6HALF_LYING;
        else if ( intrinsicsString.compare("s6-lying") == 0 )
            intrinsicsName = Mapper::S6_LYING;
        else if ( intrinsicsString.compare("s6-blender") == 0 )
            intrinsicsName = Mapper::S6_BLENDER;
        else if ( intrinsicsString.compare("iphone6-blender") == 0 )
            intrinsicsName = Mapper::IPHONE6_BLENDER;
        else if ( intrinsicsString.compare("duygu0") == 0 )
            intrinsicsName = Mapper::DUYGU0;
        else
        {
            if ( intrinsicsString.compare("kinect-aron") != 0 )
                std::cerr << "[" << __func__ << "]: " << "using default intrinsics of KINECT_ARON, could not parse " << intrinsicsString << std::endl;
            intrinsicsName = Mapper::INTRINSICS::KINECT_ARON;
            ret = EXIT_FAILURE;
        }
    } //... if "--intr"
    else
    {
        ret = EXIT_FAILURE;
    }

    mapper = Mapper( intrinsicsName );
    return ret;
} //...parseMapper()

int parseFrameIds( FrameIdsT& frameIds, int argc, const char** argv, std::string const& flag, bool required )
{
    std::vector<unsigned> tmpFrameIds;
    if ( console::parse_x_arguments(argc,argv,flag.c_str(),tmpFrameIds) < 0 )
    {
        if ( required )
            std::cerr << "[" << __func__ << "]: " << "you should specify the input frame ids by " << flag  << " start,end" << std::endl;
        return EXIT_FAILURE;
    }
    else
        frameIds.insert( frameIds.end(), tmpFrameIds.begin(), tmpFrameIds.end() );
    return EXIT_SUCCESS;
} //...parseFrameIds()

int parseImgPaths( std::vector<std::string>& imgPaths, int argc, const char** argv, const FrameIdsT& frameIds, bool allowDefault, std::string* outPattern) {
    std::string imgPattern("color_%05d.ppm");
    bool doRead = allowDefault;
    if (!console::parse_arg(argc,argv,"--img-pattern",imgPattern)) {
        if (!allowDefault) {
            std::cerr << "[" << __func__ << "]: " << "you should specify the input image pattern by --img-pattern" << std::endl;
            return EXIT_FAILURE;
        }
    } else
        doRead = true;

    if (doRead) {
        char imgPath[2048];
        for (auto const& frameId : frameIds) {
            sprintf( imgPath, imgPattern.c_str(), frameId );
            imgPaths.push_back( imgPath );
        }
    } //...ifDoRead

    if (outPattern)
        *outPattern = imgPattern;

    return EXIT_SUCCESS;
} //...parseImgPaths()

// --depth-pattern
int parseDepthPaths(std::vector<std::string>& depthPaths, int argc, const char** argv, const FrameIdsT& frameIds, bool allowDefault) {
    std::string depthPattern("depth_%05d.ppm");
    bool        doRead = allowDefault;
    if (!console::parse_arg(argc,argv,"--depth-pattern",depthPattern)) {
        if (!allowDefault) {
            std::cerr << "[" << __func__ << "]: " << "you should specify the input image pattern by --depth-pattern" << std::endl;
            return EXIT_FAILURE;
        }
    } else
        doRead = true;

    if (doRead) {
        char depthPath[2048];
        for (auto const& frameId : frameIds) {
            sprintf( depthPath, depthPattern.c_str(), frameId );
            depthPaths.push_back( depthPath );
        }
    } //...ifDoRead

    return EXIT_SUCCESS;
} //...parseImgPaths()


int parseGroupedTracks(GroupedTracks2d &groupedTracks2d, int argc, const char** argv, std::string *datPathArg, TrackId *const maxTrackIdArg) {
    groupedTracks2d.clear();

    int ret = EXIT_SUCCESS;

    // --dat
    std::string datPath;
    if ((EXIT_SUCCESS == ret) &&
        (!console::parse_arg(argc, argv, "--dat", datPath))) {
        std::cerr << "[" << __func__ << "] " << "need \"--dat\" flag as input" << std::endl;
        ret = EXIT_FAILURE;
    }

    int64_t filtered {0};

    TrackId maxTrackId{0ul}; // used in jicp
    if (EXIT_SUCCESS == ret) {
        GroupedTracks2d allTracks2d;
        if (EXIT_SUCCESS == (ret=tracking::io::readSegmentedTracks(datPath, allTracks2d)))
            std::cout << "[" << __func__ << "] " << "tracks read..." << std::endl;

        // filter input
        for (auto const& groupIdAndLinIds : allTracks2d.getGroups()) {
            GroupId const   & groupId = groupIdAndLinIds.first;
            auto const      & linIds  = groupIdAndLinIds.second;
            for (LinId const& linId : linIds) {
                Track2D const& track2D = allTracks2d.getTrack(linId);
                if (track2D.size() < 3) {
                    ++filtered;
                    continue;
                }
                groupedTracks2d.addTrack(track2D, groupId);
            }
        }
        std::cout << "[" << __func__ << "] working with " << groupedTracks2d.size() << " tracks after filtering"
                  << std::endl;
        maxTrackId = allTracks2d.getMaxTrackId();
    }

    if (filtered)
        std::cout << "[" << __FILE__ << ":" << __LINE__ << "] " << "filtered from length: " << filtered << std::endl;

    if (datPathArg)
        *datPathArg = datPath;
    if (maxTrackIdArg)
        *maxTrackIdArg = maxTrackId;
    return ret;
} //...parseGroupedTracks()

} //...ns tracking
