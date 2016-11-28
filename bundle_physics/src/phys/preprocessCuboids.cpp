//
// Created by bontius on 03/05/16.
//

#include "tracking/annot/cuboidFwDecl.h"
#include "tracking/annot/cuboid.h"
#include "tracking/phys/typedefsGeometry.h"
#include <map>
#include <iostream>

namespace tracking {
  namespace bundle_physics {
    void orientCuboids(CuboidsT &cuboids) {
        for (auto &cuboid : cuboids) {
            std::shared_ptr<QuaternionT> prevQ(nullptr);
            for (Cuboid::StatesT::value_type const& frameIdAndState : cuboid.getStates()) {
                if (!frameIdAndState.second.hasPose())
                    continue;

                if (prevQ) {
                    Scalar dot = prevQ->coeffs().dot(frameIdAndState.second.getPose().coeffs());
//                    std::cout << "[" << __func__ << "] " << "[frame" << frameIdAndState.first << "]: "
//                              << "prevQ: " << prevQ->coeffs().transpose()
//                              << ", q: " << frameIdAndState.second.getPose().coeffs().transpose()
//                              << ", dot: " << dot
//                              << std::endl;
                   if (dot < Scalar(0.)) {
//                       std::cout << "[" << __func__ << "] " << "flipping" << frameIdAndState.second.getPose().coeffs().transpose() << " ->  ";
                       cuboid.setPose(frameIdAndState.first,QuaternionT{frameIdAndState.second.getPose().coeffs() * Scalar(-1.)});
                       std::cout << cuboid.getPose(frameIdAndState.first).coeffs().transpose() << std::endl;
                   }
                } else {
                    prevQ = std::make_shared<QuaternionT>(frameIdAndState.second.getPose());
                }
                *prevQ = frameIdAndState.second.getPose();
            }
        }
    }
  } //...ns bundle_phsyics
} //...ns tracking
