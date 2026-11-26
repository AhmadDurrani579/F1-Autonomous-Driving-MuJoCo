#pragma once

#include "MPCConfig.h"
#include <casadi/casadi.hpp>
#include "MPCCost.h"
#include "../models/KinematicBicycleModel.h"

#include <limits>
#include <vector>

struct MPCResult {
    bool success = false;

    BicycleControl control{0.0, 0.0};

    std::vector<BicycleState> predictedStates;
    std::vector<BicycleControl> predictedControls;

    double totalCost = 0.0;
};

class MPCOptimizer {
public:
    explicit MPCOptimizer(const MPCConfig& config);

    MPCResult solve(
        const BicycleState& initialState,
        const std::vector<MPCReference>& references,
        double previousSteering = 0.0,
          double measuredSteering =
              std::numeric_limits<double>::quiet_NaN()
    );

private:
    MPCConfig config_;
    casadi::Function solver_;
    std::vector<double> previousControls_;
};
