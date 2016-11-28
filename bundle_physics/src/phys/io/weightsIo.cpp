//
// Created by bontius on 28/03/16.
//

#include "tracking/common/util/impl/parse.hpp"
#include "tracking/phys/io/weightsIo.h"
#include "tracking/phys/weights.h"
#include <iostream>

namespace tracking {
  namespace bundle_physics {
    namespace io {
      std::pair<Weights,int> readWeights(int argc, const char **argv) {
          Weights weights;

          // vis
          weights.doVis = !console::find_switch(argc, argv, "--no-vis");

          // init, solve pos or solve
          if (console::find_switch(argc, argv, "--no-solve"))
              weights.solveFlags = bundle_physics::Weights::NO_SOLVE;
          else if (console::find_switch(argc, argv, "--solve-pos"))
              weights.solveFlags = bundle_physics::Weights::SOLVE_POS;
          else
              weights.solveFlags = bundle_physics::Weights::SOLVE;
          if (console::find_switch(argc, argv, "--re-solve"))
              weights.solveFlags |= bundle_physics::Weights::RE_SOLVE;

          // fps
          if (!console::parse_arg(argc, argv, "--fps", weights.fps)) {
              std::cerr << "Error: need fps as input with \"--fps\"" << std::endl;
              return std::make_pair(Weights(),EXIT_FAILURE);
          }

          // weights
          {
              console::parse_arg(argc, argv, "--weight-observed-x", weights.observedPosWeight);
              console::parse_arg(argc, argv, "--weight-observed-q", weights.observedPoseWeight);
              console::parse_arg(argc, argv, "--weight-velocity", weights.velocityWeight);
              console::parse_arg(argc, argv, "--weight-conservation-lin", weights.conservationWeight);
              console::parse_arg(argc, argv, "--weight-cor", weights.corWeight);
              console::parse_arg(argc, argv, "--weight-ke", weights.keWeight);

              console::parse_arg(argc, argv, "--weight-coll-point", weights.collPointWeight);
              console::parse_arg(argc, argv, "--weight-points-uv", weights.pointsUvWeight);
              console::parse_arg(argc, argv, "--weight-min-momentum", weights.minMomentumWeight);
              console::parse_arg(argc, argv, "--weight-gravity-down", weights.gravityDownWeight);
          }

          if (console::find_switch(argc, argv, "--fix-gravity"))
              weights.fixFlags |= bundle_physics::Weights::FIX_FLAGS::FIX_GRAVITY;
          if (console::find_switch(argc, argv, "--fix-pose"))
              weights.fixFlags |= bundle_physics::Weights::FIX_FLAGS::FIX_POSE;
          if (console::find_switch(argc, argv, "--fix-momenta"))
              weights.fixFlags |= bundle_physics::Weights::FIX_MOMENTA;

#         warning Changed on 20160428
          weights.poseIntegralSteps = 0.5;
          return {std::move(weights),EXIT_SUCCESS};
      } //...readWeights()
    } //...ns io
  } //...ns bundle_physics
} //...ns tracking