//
// Created by bontius on 13/01/16.
//

#ifndef TRACKVIDEO_PHYS_INERTIAESTIMATION_HPP
#define TRACKVIDEO_PHYS_INERTIAESTIMATION_HPP

//#include "trackVideo/annot/cuboid.h"
#include "tracking/phys/typedefs.h"
#include <iostream>

namespace tracking {
namespace bundle_physics {

namespace internal {
template<typename T, typename T2>
inline void getIBoxHollow(T const* mass, const T2 *const sqrSize, T *I) {
    I[0] = mass[0] * (sqrSize[0] + 2. * (sqrSize[1] + sqrSize[2])) / T(3.);
    I[1] = mass[0] * (sqrSize[1] + 2. * (sqrSize[0] + sqrSize[2])) / T(3.);
    I[2] = mass[0] * (sqrSize[2] + 2. * (sqrSize[0] + sqrSize[1])) / T(3.);
} //...getIBoxHollow()

template<typename T, typename T2>
inline void getInvIBoxHollow(T const* const mass, const T2 *const sqrSize, T *invI) {
    invI[0] = T(3.) / (T(sqrSize[0] + 2. * (sqrSize[1] + sqrSize[2])) * mass[0]);
    invI[1] = T(3.) / (T(sqrSize[1] + 2. * (sqrSize[0] + sqrSize[2])) * mass[0]);
    invI[2] = T(3.) / (T(sqrSize[2] + 2. * (sqrSize[0] + sqrSize[1])) * mass[0]);
} //...getInvIBoxHollow()

template<typename T, typename T2>
inline void getIBox(const T *mass, const T2 *const sqrSize, T *I) {
    I[0] = mass[0] * T(sqrSize[1] + sqrSize[2]) / T(12.);
    I[1] = mass[0] * T(sqrSize[0] + sqrSize[2]) / T(12.);
    I[2] = mass[0] * T(sqrSize[0] + sqrSize[1]) / T(12.);
} //...getIBox()

template<typename T, typename T2>
inline void getInvIBox(T const* const mass, T2 const* const sqrSize, T *invI) {
    invI[0] = T(12.) / (T(sqrSize[1] + sqrSize[2]) * mass[0]);
    invI[1] = T(12.) / (T(sqrSize[0] + sqrSize[2]) * mass[0]);
    invI[2] = T(12.) / (T(sqrSize[0] + sqrSize[1]) * mass[0]);
} //...getInvIBox()

template<typename T, typename T2>
inline void getIEllipsoid(T const* mass, T2 const* const sqrSize, T *I) {
    I[0] = mass[0] * T(sqrSize[1] + sqrSize[2]) / T(20.); // m(b/2)^2 / 5 = m b^2/4 / 5 = b^2/
    I[1] = mass[0] * T(sqrSize[0] + sqrSize[2]) / T(20.);
    I[2] = mass[0] * T(sqrSize[0] + sqrSize[1]) / T(20.);
    std::cout << "[" << __func__ << "] " << "ellipsoid..." << std::endl;
} //...getIEllipsoid()

template<typename T, typename T2>
inline void getInvIEllipsoid(const T *const mass, const T2 *const sqrSize, T *invI) {
    invI[0] = T(20.) / (T(sqrSize[1] + sqrSize[2]) * mass[0]);
    invI[1] = T(20.) / (T(sqrSize[0] + sqrSize[2]) * mass[0]);
    invI[2] = T(20.) / (T(sqrSize[0] + sqrSize[1]) * mass[0]);
    std::cout << "[" << __func__ << "] " << "ellipsoid..." << std::endl;
} //...getInvIEllipsoid()
} //...ns internal

template<typename T, typename T2>
inline void getI(SHAPE const shape, T const* mass, T2 const* const sqrSize, T *I) {
    if (shape == BOX) {
        internal::getIBox(mass, sqrSize, I);
    } else if (shape == ELLIPSOID) {
        internal::getIEllipsoid(mass, sqrSize, I);
    } else if (shape == BOX_HOLLOW) {
        internal::getIBoxHollow(mass, sqrSize, I);
    } else {
        std::cerr << "[getI] TODO: other shapes" << std::endl;
        throw new std::runtime_error("getI for shape" + std::to_string(shape));
    }
} //...getI()

template<typename T, typename T2>
inline void getInvI(SHAPE const shape, T const* const mass, T2 const* const sqrSize, T *invI) {
    if (shape == BOX) {
        internal::getInvIBox(mass, sqrSize, invI);
    } else if (shape == ELLIPSOID) {
        internal::getInvIEllipsoid(mass, sqrSize, invI);
    } else if (shape == BOX_HOLLOW) {
        internal::getInvIBoxHollow(mass, sqrSize, invI);
    } else {
        std::cerr << "[getInvI] TODO: other shapes" << std::endl;
        throw new std::runtime_error("getI for shape" + std::to_string(shape));
    }
} //...getInvI()

} //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_INERTIAESTIMATION_HPP
