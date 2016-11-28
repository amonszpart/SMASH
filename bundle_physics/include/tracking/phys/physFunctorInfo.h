//
// Created by bontius on 29/03/16.
//

#ifndef TRACKVIDEO_PHYS_COSTFUNCTORS_H
#define TRACKVIDEO_PHYS_COSTFUNCTORS_H

#include <tracking/common/typedefs.h>
#include "ceres/ceres.h"
#include "ceres/functorInfo.h"

namespace tracking {
  namespace bundle_physics {
    class PhysFunctorInfo : public ceres::FunctorInfo
    {
        public:
            PhysFunctorInfo(std::string const& name_arg, ceres::CostFunction* const fn, std::vector<double*> data, ceres::ResidualBlockId const& residualBlockId)
                : FunctorInfo(name_arg,fn,data), _residualBlockId(residualBlockId) {}
            virtual ~PhysFunctorInfo() = default;

            inline ceres::ResidualBlockId getResidualBlockId() const { return _residualBlockId; }

        protected:
            ceres::ResidualBlockId const _residualBlockId;
    }; //...class MyFunctorInfo

    class TrackPhysFunctorInfo : public PhysFunctorInfo {
        public:
            TrackPhysFunctorInfo(std::string const &name_arg, ceres::CostFunction *const fn, std::vector<double*> const& data,
                TrackId const trackId, ceres::ResidualBlockId const &residualBlockId)
                : PhysFunctorInfo(name_arg, fn, data, residualBlockId), _trackIds({trackId}) {}

            TrackPhysFunctorInfo(std::string const &name_arg, ceres::CostFunction *const fn, std::vector<double*> const& data,
                std::vector<TrackId> const& trackIds, ceres::ResidualBlockId const &residualBlockId)
                : PhysFunctorInfo(name_arg, fn, data, residualBlockId), _trackIds(trackIds) { }

            inline TrackId getTrackId(int const id = 0) const { return _trackIds.at(id); }

            inline std::vector<TrackId> const& getTrackIds() const { return _trackIds; }

            virtual ~TrackPhysFunctorInfo() = default;

        protected:
            std::vector<TrackId> const _trackIds;
    }; //...class TrackPhysFunctorInfo

    class CompactnessPhysFunctorInfo : public PhysFunctorInfo {
        public:
            using PhysFunctorInfo::PhysFunctorInfo;
   };

  } //...ns bundle_phsyics
} //...ns tracking

#include "tracking/phys/physFunctorInfoFwDecl.h"

#endif //TRACKVIDEO_PHYS_COSTFUNCTORS_H
