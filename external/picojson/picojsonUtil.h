//
// Created by bontius on 29/12/15.
//

#ifndef TRACKVIDEO_PICOJSONUTIL_H
#define TRACKVIDEO_PICOJSONUTIL_H

#include "picojson/picojson.h"
#include <array>

namespace picojson {

template <typename _Vector3>
inline picojson::value vector3ToPicojsonObject( const _Vector3& v ) {
    picojson::object obj;
    obj["x"] = picojson::value(v(0));
    obj["y"] = picojson::value(v(1));
    obj["z"] = picojson::value(v(2));
    return picojson::value(obj);
}

template <typename _Vector3>
inline picojson::value vector3ToArray( const _Vector3& v ) {
    return picojson::value( picojson::array{picojson::value(v(0)),picojson::value(v(1)),picojson::value(v(2))} );
} //...vector3ToArray

template <typename _Vector4>
inline picojson::value vector4ToArray(_Vector4 const& v) {
    return picojson::value(picojson::array{picojson::value(v(0)),
                                           picojson::value(v(1)),
                                           picojson::value(v(2)),
                                           picojson::value(v(3))});
} //...vector3ToArray

template <typename Scalar>
inline picojson::value array3ToPicojsonObject( const std::array<Scalar,3>& v ) {
    picojson::object obj;
    obj["x"] = picojson::value(v[0]);
    obj["y"] = picojson::value(v[1]);
    obj["z"] = picojson::value(v[2]);
    return picojson::value(obj);
}

template <typename _Vector4>
inline picojson::value vector4ToPicojsonObject(_Vector4 const& v) { picojson::object obj; obj["x"] = picojson::value(v(0)); obj["y"] = picojson::value(v(1)); obj["z"] = picojson::value(v(2)); obj["w"] = picojson::value(v(3)); return picojson::value(obj); }

template <class _Vector4>
inline int parseVector4(_Vector4& pose, const picojson::object& object) {
    for (picojson::object::const_iterator it = object.begin(); it != object.end(); ++it) {
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
inline int parseVector3(_Vector3& angVel, const picojson::object& object) {
    for (picojson::object::const_iterator it = object.begin(); it != object.end(); ++it) {
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

template <class _Vector3>
inline int parseArray3(_Vector3& v, picojson::array const& object) {
    int i = 0;
    for ( picojson::array::const_iterator it = object.begin(); it != object.end(); ++it, ++i ) {
        v(i) = std::atof( it->to_str().c_str() );
    }

    return EXIT_SUCCESS;
} //...parseVector3

inline Eigen::Vector4f parseArray4(picojson::array const& object) {
    Eigen::Vector4f v {Eigen::Vector4f::Zero()};
    int i = 0;
    for (picojson::array::const_iterator it = object.begin(); it != object.end(); ++it, ++i) {
        v(i) = std::atof(it->to_str().c_str());
    }

    return v;
} //...parseVector3

} //...ns picojson

#endif //TRACKVIDEO_PICOJSONUTIL_H
