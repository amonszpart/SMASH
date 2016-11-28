//
// Created by bontius on 30/12/15.
//

#ifndef TRACKVIDEO_PHYS_BUNDLEWITHPHYSICSRESULT_H
#define TRACKVIDEO_PHYS_BUNDLEWITHPHYSICSRESULT_H

#include "tracking/phys/parabola.h"
#include "tracking/phys/initialize/parabola2d.h"
#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/annot/cuboid.h"
#include "tracking/common/groupedTracksFwDecl.h"
#include <map>

namespace tracking {
namespace bundle_physics {
/** \brief Stores impulse and relative collision point of the collision, and Cuboid states (PoseLoc) before and after. */
struct CollisionInfo {
    public:
        //! Stores \ref PoseLoc for each cuboid at a time point.
        typedef std::map<CuboidId, PoseLoc> CuboidStatesT;

        std::pair<CuboidStatesT, CuboidStatesT> states;         //!< before, after
        Vector3                                 impulse;        //!< \f$ J\mathbf{n} \f$
        Vector3                                 relCollPoint;   //!< Relative collision vector in world space w.r.t. object A (first object of the two in the collision)
        ceres::CeresScalar                      cor;            //!< Coefficient of restitution of this pair of objects.
}; //...CollisionInfo


struct PerfLog {
        void log(ceres::Solver::Summary const& summary, PhysIndexer const& indexer, CollId const collId,
                 std::vector<CuboidId> const& participants);
        void clear();

        std::vector<Scalar> const& getEnergies() const;
        std::vector<Scalar> const& getRunTimes() const;
        std::vector<Scalar> const& getMassEstimates() const;
        void dump(std::string const& evalPath, BundleWithPhysicsResult const& result, Weights const& weights) const;

    protected:
        std::vector<Scalar> _energies;
        std::vector<Scalar> _runTimes;
        std::vector<Scalar> _massEstimates;
        std::vector<ceres::Solver::Summary> _summaries;
}; //...class PerfLog

/** \brief Stores an optimization output, or initialization for alternation or iteration. */
struct BundleWithPhysicsResult {
    public:
        //typedef Cuboid::Vector3           Vector3;        //!< \copydoc Cuboid::Vector3
        //typedef Cuboid::Scalar            Scalar;         //!< \copydoc Cuboid::Scalar
        typedef std::map<PartId, Vector3> PartDynamics;   //!< Stores abstract pose info (momentum) for a trajectory (part).

        Vector3                          gravity;   //!< Rotated gravity vector
        Scalar                           a, rotX, rotY1;
        CuboidsPtrT cuboids;   //!< Cuboids with size, mass, timed position, pose, linVel and angVel
        Parabolas                        parabolas; //!< Position info
        std::map<CuboidId, PartDynamics> momenta;   //!< Todo Pose info
        Scalar mass; // initial mass of other object

        std::map<CollId, Scalar>        collTimes; //!< {collId,collTime}, where collId is between partId = collId and partId == collId+1
        std::map<CollId, CollisionInfo> collisions; //!< Cuboid states, impulse and relative collision point of the collisions.

        std::shared_ptr<GroupedTracks3d> tracks3d;  //!< Output points
        Tracks3DPtr points3d;  //!< Contains points in object space (timeless)

        ceres::TerminationType terminationType;


        bool hasParabola(const CuboidId cuboidId, const PartId partId) const;
        bool hasMomentum(const CuboidId cuboidId, const PartId partId) const;
        void clear();

        inline Parabola2ds const& getCircles() const { return _circles; }
        inline void               setCircles(Parabola2ds const &circles) { _circles = circles; }
        PerfLog const& getLog() const;
        PerfLog & getLog();
    protected:
        Parabola2ds  _circles;
        PerfLog      _log;

}; //...BundleWithPhysicsResult

} //...ns bundle_physics
} //...ns tracking

#endif // TRACKVIDEO_PHYS_BUNDLEWITHPHYSICSRESULT_H
