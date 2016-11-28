//
// Created by bontius on 08/07/16.
//

#include "tracking/phys/physIndexer.h"

namespace tracking {
namespace bundle_physics {

PhysIndexer::CeresScalar const*
PhysIndexer::getParabolaFreeParamsConst(const CuboidId cuboidId, const PartId partId) const {
    if (!_inited)
        throw new PhysIndexer_NotInitedException("Please call allocate()");

    if (partId >= _nParabolas / _nCuboids)
        throw new PhysIndexer_GetParabolaTransform_OverindexingException{
            "We don't have this many parabolas for this cuboid"};

    auto iter = _aliases[FREE_ANGLE_ALIAS].find({cuboidId,partId});
    if (_aliases[FREE_ANGLE_ALIAS].end() != iter) {
        return iter->second;
    } else
        return &(_freeAngleAndLinParams.at((cuboidId * _nParts + partId) * PARABOLA_FREE_STRIDE));
}

PhysIndexer::CeresScalar*
PhysIndexer::getParabolaFreeParams(const int cuboidId, const int partId) {
    return const_cast<CeresScalar*>(static_cast<PhysIndexer const&>(*this).getParabolaFreeParamsConst(cuboidId, partId));
}

PhysIndexer::CeresScalar const*
PhysIndexer::getParabolaRotationFreeConst(const CuboidId cuboidId, const PartId partId) const {
    if ( !_inited )
        throw new PhysIndexer_NotInitedException("Please call allocate()");
    if ( partId >= _nParabolas / _nCuboids )
        throw new PhysIndexer_GetParabolaTransform_OverindexingException("We don't have this many parabolas for this cuboid");
    return getParabolaFreeParamsConst(cuboidId,partId) + PARABOLA_FREE_ANGLE_OFFSET;
}

PhysIndexer::CeresScalar*
PhysIndexer::getParabolaRotationFree(const CuboidId cuboidId, const PartId partId)
{ return const_cast<CeresScalar*>(static_cast<PhysIndexer const&>(*this).getParabolaRotationFreeConst(cuboidId,partId)); }


} //...ns bundle_phsyics
} //...ns tracking
