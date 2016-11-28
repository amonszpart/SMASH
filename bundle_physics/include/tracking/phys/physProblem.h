//
// Created by bontius on 07/07/16.
//

#ifndef TRACKVIDEO_PHYS_PHYSPROBLEM_H
#define TRACKVIDEO_PHYS_PHYSPROBLEM_H


#include "tracking/phys/consts.h"
#include "tracking/phys/physIndexer.h"
#include "ceres/functorInfo.h"
#include "ceres/problem.h"

namespace tracking {
namespace bundle_physics {

struct PhysProblem {
    public:
        PhysProblem(std::shared_ptr<ceres::Problem> problem) : _problem( problem ? problem
                                                                                 : std::make_shared<ceres::Problem>()) {}
        PhysProblem(PhysProblem&&) = default;
        PhysProblem& operator=(PhysProblem&&) = default;
        virtual ~PhysProblem() = default;
        PhysProblem(PhysProblem const&) = delete;
        PhysProblem& operator=(PhysProblem const&) = delete;

        std::shared_ptr<ceres::Problem> const& getProblemPtrRef() const { return _problem; }
        std::shared_ptr<ceres::Problem>      & getProblemPtrRef()       { return _problem; }
        std::shared_ptr<ceres::Problem>        getProblemPtr   ()       { return _problem; }
                        ceres::Problem       & getProblem      ()       { return *_problem; }
                        ceres::Problem  const& getProblem      () const { return *_problem; }

        Consts const& getConsts() const { return *consts; }
        PhysIndexer& getIndexer() { return *indexer; }

        ceres::FunctorInfosT const& getCostFunctors() const { return costFunctors; }
        ceres::FunctorInfosT & getCostFunctors()  { return costFunctors; }

        std::vector<CuboidId> const& getParticipants() const { return participants; }

        ceres::LossFunctionWrapper* getPoseLoss() { return poseLoss; }

        void setConsts(Consts* consts) { PhysProblem::consts.reset(consts); }
        void setIndexer(PhysIndexer* indexer) { PhysProblem::indexer.reset(indexer); }
        void setPoseLoss(ceres::LossFunctionWrapper *const poseLoss) { PhysProblem::poseLoss = poseLoss; }

    private:
        std::shared_ptr<ceres::Problem> _problem = nullptr;
        std::unique_ptr<Consts> consts = nullptr;
        std::unique_ptr<PhysIndexer> indexer = nullptr;
        ceres::FunctorInfosT costFunctors;
        std::vector<CuboidId> participants = {0,1};
        ceres::LossFunctionWrapper* poseLoss = nullptr; // weak_ptr
};

} //...ns bundle_physics
} //...ns tracking

#endif //TRACKVIDEO_PHYS_PHYSPROBLEM_H
