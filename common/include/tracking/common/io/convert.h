#ifndef AM_CONVERT_H
#define AM_CONVERT_H

namespace tracking
{
    int convertPPM( int argc, const char** argv );
    int convertTracksToBinary( int argc, const char** argv );
    int maskTracks( int argc, const char** argv );
    int convertPng2Ppm( int argc, const char** argv );
    int convertDmap( int argc, const char** argv );
    int maskPpm(int argc, const char ** argv);
} //...tracking

#endif // AM_CONVERT_H
