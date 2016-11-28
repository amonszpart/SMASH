//
// Created by bontius on 15/06/15.
//

#ifndef CCDPHYSICSDEMO_IO_PICOJSON_HPP
#define CCDPHYSICSDEMO_IO_PICOJSON_HPP

#include "picojson/picojson.h"

namespace picojson
{
    template <class _Vector4>
    inline int parseVector4( _Vector4& pose, const picojson::object& object )
    {
        typedef typename _Vector4::Scalar Scalar;

        for ( picojson::object::const_iterator it = object.begin(); it != object.end(); ++it )
        {
            if ( it->first.compare("x") == 0 )
                pose(0) = std::atof( it->second.to_str().c_str() );
            else if ( it->first.compare("y") == 0 )
                pose(1) = std::atof( it->second.to_str().c_str() );
            else if ( it->first.compare("z") == 0 )
                pose(2) = std::atof( it->second.to_str().c_str() );
            else if ( it->first.compare("w") == 0 )
                pose(3) = std::atof( it->second.to_str().c_str() );
            else
                std::cout << "[" << __func__ << "]: could not parse entry by key " << it->first << std::endl;
        }

        return EXIT_SUCCESS;
    } //...parseVector4

    template <class _Vector3>
    inline int parseVector3( _Vector3& angVel, const picojson::object& object )
    {
        typedef typename _Vector3::Scalar Scalar;

        for ( picojson::object::const_iterator it = object.begin(); it != object.end(); ++it )
        {
            if ( it->first.compare("x") == 0 )
                angVel(0) = std::atof( it->second.to_str().c_str() );
            else if ( it->first.compare("y") == 0 )
                angVel(1) = std::atof( it->second.to_str().c_str() );
            else if ( it->first.compare("z") == 0 )
                angVel(2) = std::atof( it->second.to_str().c_str() );
            else
                std::cout << "[" << __func__ << "]: could not parse entry by key " << it->first << std::endl;
        }

        return EXIT_SUCCESS;
    } //...parseVector3
} //...ns physacq

#endif //CCDPHYSICSDEMO_IO_PICOJSON_HPP
