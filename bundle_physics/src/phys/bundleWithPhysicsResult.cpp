//
// Created by bontius on 30/12/15.
//

#include "tracking/phys/partUtil.h" // getParticipant()
#include "tracking/phys/bundleWithPhysicsResult.h"
#include "tracking/phys/weights.h"
#include <fstream>

namespace tracking {
namespace bundle_physics {

bool BundleWithPhysicsResult::hasParabola(const CuboidId cuboidId, const PartId partId) const
{
    return parabolas.find(cuboidId) != parabolas.end() &&
           parabolas.at(cuboidId).find(partId) != parabolas.at(cuboidId).end();
}

bool BundleWithPhysicsResult::hasMomentum(const CuboidId cuboidId, const PartId partId) const
{
    return momenta.find(cuboidId) != momenta.end() &&
           momenta.at(cuboidId).find(partId) != momenta.at(cuboidId).end();
}

void BundleWithPhysicsResult::clear()
{
    collTimes.clear();
    collisions.clear();
    momenta.clear();
    if (cuboids)
        cuboids->clear();
    parabolas.clear();
}

PerfLog const& BundleWithPhysicsResult::getLog() const { return _log; }
PerfLog      & BundleWithPhysicsResult::getLog() { return _log; }

void PerfLog::clear() {
    _energies.clear();
    _runTimes.clear();
    _massEstimates.clear();
}

std::vector<Scalar> const& PerfLog::getEnergies() const { return _energies; }

std::vector<Scalar> const& PerfLog::getRunTimes() const { return _runTimes; }

std::vector<Scalar> const& PerfLog::getMassEstimates() const { return _massEstimates; }

void PerfLog::log(ceres::Solver::Summary const& summary, PhysIndexer const& indexer, CollId const collId,
                  std::vector<CuboidId> const& participants) {
    if (!_energies.size())
        _energies.push_back(summary.initial_cost);
    _energies.push_back(summary.final_cost);
    _runTimes.push_back(summary.total_time_in_seconds);
    _massEstimates.push_back(*indexer.getMassConst(getParticipant(1,collId, participants)));
    _summaries.push_back(summary);
}

void PerfLog::dump(std::string const& evalPath, BundleWithPhysicsResult const& result, Weights const& weights) const {
    std::ofstream ofs(evalPath, std::ofstream::out | std::ofstream::app);
    if (result.collTimes.size()) {
        std::cout << "mass: " << result.cuboids->at(1).getMass() << ","
                  << "cor: " << result.collisions.at(0).cor << ","
                  << "tc: " << result.collTimes.at(0) << ",";
    }
    ofs << result.cuboids->at(1).getMass() << ","
        << result.collisions.at(0).cor << ","
        << result.collTimes.at(0) << ",";

    ofs << weights.observedPosWeight << ","
        << weights.observedPoseWeight << ","
        << weights.velocityWeight << ","
        << weights.conservationWeight << ","
        << weights.gravityDownWeight << ","
        << weights.collPointWeight << ","
        << weights.pointsUvWeight << ","
        << weights.minMomentumWeight << ","
        << weights.fps << ","
        << weights.poseIntegralSteps << ",";
    std::cout << "energies: ";
    for ( size_t i = 0; i != _energies.size(); ++i )
    {
        std::cout << _energies[i];
        ofs << _energies[i];
        if ( i != _energies.size() - 1 )
        {
            ofs << ",";
            std::cout << ",";
        }
    }
    ofs << ",";
    std::cout << ", times: ";
    for (size_t i = 0; i != _runTimes.size(); ++i )
    {
        std::cout << _runTimes[i];
        ofs << _runTimes[i];
        if ( i != _runTimes.size() - 1 )
        {
            ofs << ",";
            std::cout << ",";
        }
    }
    ofs << ",";
    std::cout << ", masses: ";
    for (size_t i = 0; i != _massEstimates.size(); ++i )
    {
        std::cout << _massEstimates[i];
        ofs << _massEstimates[i];
        if ( i != _massEstimates.size() - 1 )
        {
            ofs << ",";
            std::cout << ",";
        }
    }
    std::cout << std::endl;
    ofs << std::endl;
    ofs.close();
}

} //...bundle_physics
} //...ns tracking