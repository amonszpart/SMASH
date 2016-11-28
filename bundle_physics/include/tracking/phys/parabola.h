//
// Created by bontius on 27/12/15.
//

#ifndef TRACKVIDEO_PHYS_PARABOLA_H
#define TRACKVIDEO_PHYS_PARABOLA_H

#include "tracking/common/eigen.h"
#include "tracking/phys/typedefs.h"
#include "tracking/common/typedefs.h"
#include "ceres/ceresUtil.h"
#include <array>
#include <map>

namespace tracking {
  namespace bundle_physics {
    struct Parabola {
            using Scalar = ceres::CeresScalar;
            Parabola()  = default;
            Parabola(Scalar const b, Scalar const s, Scalar const rotY0, Scalar const*const translation)
                : b(b), s(s), rotY0(rotY0), _transl {translation[0],translation[1],translation[2]} {}

            Scalar b, s, rotY0;

            inline void setTranslation(const Scalar t[3]) { _transl = { t[0], t[1], t[2] }; }
            inline const std::array<Scalar, 3> &getTranslation() const { return _transl; };

            Eigen::Matrix<Scalar,3,1> getPosition(FrameId const frameId, Scalar const rotX, Scalar const rotY1, Scalar const a, Scalar const collTime) const;
            static bool similar(Parabola const& first, Parabola const& other, double const eps = kSmallDiff);

        protected:
            std::array<Scalar, 3> _transl;
    }; //...struct Parabola

    /** Stores information about a constant trajectory of a Cuboid between any two collisions (i.e. parts). */
    typedef std::map<CuboidId, std::map<PartId, Parabola> > Parabolas;
}
} //...ns tracking

#include "tracking/phys/energyTerms/parabolaTerm.h"
#include "tracking/phys/physIndexer.h"

namespace tracking {
namespace bundle_physics {
inline Eigen::Matrix<Parabola::Scalar, 3, 1>
Parabola::getPosition(FrameId const frameId, Scalar const rotX, Scalar const rotY1, Scalar const a,
                      Scalar const collTime) const {
    Eigen::Matrix<Scalar, 3, 1> x;

    Eigen::Matrix<Scalar, 3, 1> freeParams;
    freeParams(PhysIndexer::PARABOLA_FREE_ANGLE_OFFSET) = rotY0;
    freeParams(PhysIndexer::PARABOLA_FREE_B_OFFSET)     = b;
    freeParams(PhysIndexer::PARABOLA_FREE_S_OFFSET)     = s;
    Eigen::Matrix<Scalar,2,1> rotG(rotX,rotY1); // gravity rotation X,Y1
    ParabolaCostFunctor::getPositionAtTime(x.data(), rotG.data(), &a, _transl.data(), freeParams.data(),
                                           frameId - collTime);
    return x;
} //...getPosition()


inline bool Parabola::similar(Parabola const& first, Parabola const& second, double const eps) {
    return (   (std::abs(first.b - second.b) < eps)
               && (std::abs(first.s - second.s) < eps)
               && (std::abs(first.rotY0 - second.rotY0) < eps));
} //...similar()


} //...ns bundle_phsyics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_PARABOLA_H
