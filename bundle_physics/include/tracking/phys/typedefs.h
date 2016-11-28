//
// Created by bontius on 26/12/15.
//

#ifndef TRACKVIDEO_PHYS_TYPEDEFS_H
#define TRACKVIDEO_PHYS_TYPEDEFS_H

namespace tracking
{
    enum SHAPE { BOX, SPHERE, ELLIPSOID, BOX_HOLLOW };

#if 1
    typedef int                  CollId; //!< Linear collision id. 0: first collision, etc... \note Can NOT be unsigned!
#else
    struct CollId
    {
            inline explicit CollId( int v ) : internal(v) {}

            operator int()    const             { return internal; }
            operator size_t() const             { return internal; }
            bool operator<=( const CollId& other ) const { return this->internal <= other.internal; }
            bool operator>=( const CollId& other ) const { return this->internal >= other.internal; }
            bool operator<( const int other ) const { return this->internal < other; }
            bool operator>( const int other ) const { return this->internal > other; }
            bool operator>=( const int other ) const { return this->internal >= other; }
            bool operator<=( const int other ) const { return this->internal <= other; }
            CollId operator+( const int other ) const { return CollId(this->internal + other); }
            CollId& operator++() { ++this->internal; return *this; }
            bool operator==( const int other ) const { return this->internal == other; }

        protected:
            int internal;
    };
#endif

    typedef int                  PartId; //!< Which part (parabola) of a cuboids trajectory.

    /** \brief Unique Cuboid identifier. */
    typedef int                  CuboidId;

    static constexpr double kSmallDiff(1.e-3);
} //...ns tracking

#endif //TRACKVIDEO_PHYS_TYPEDEFS_H
