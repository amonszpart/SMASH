#ifndef TV_OPTPARAMS_H
#define TV_OPTPARAMS_H

namespace tracking
{
    struct OptParams
    {
            OptParams( const int maxIterationsArg = 2000, const float trackDistThreshArg = 5.f )
                : maxIterations  ( maxIterationsArg   )
                , trackDistThresh( trackDistThreshArg )
            {}

            int     maxIterations;
            float   trackDistThresh;
    }; // OptParams()
} //...ns tracking

#endif // TV_OPTPARAMS_H
