#ifndef VIS2D_H
#define VIS2D_H

namespace tracking
{
    //int showTracks( const std::string& imgPattern, const Tracks& tracks );
    int showTracks2D( const std::string& imgPattern, const tracking::Tracks2D& tracks, const FrameId startFrame, const FrameId endFrame, const int nTracks = 0 );
}

#endif // VIS2D_H
