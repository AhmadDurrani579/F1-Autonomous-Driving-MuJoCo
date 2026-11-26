#include "../src/controllers/MPCOptimizer.h"
#include "../src/controllers/MPCConstraints.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    MPCConfig config;
    MPCOptimizer optimizer(config);
    MPCConstraints constraints(config);

    BicycleState initialState{0.0, 0.0, 0.0, 1.0};

    std::vector<MPCReference> references;

    for (int k = 0; k < config.horizon; ++k) {
        double t = (k + 1) * config.dt;
        references.push_back({2.0 * t, 0.0, 0.0, 2.0});
    }

    MPCResult result = optimizer.solve(initialState, references);

    if (!result.success ||
        result.predictedStates.size() !=
            static_cast<size_t>(config.horizon)) {
        std::cerr << "STEP 292: FAILED — Invalid solution\n";
        return 1;
    }

    bool allStatesFeasible = true;
    double maxSpeed = 0.0;
    double maxLateralError = 0.0;

    for (size_t k = 0; k < result.predictedStates.size(); ++k) {
        const BicycleState& state = result.predictedStates[k];

        // Zero control is used to check state feasibility only.
        bool feasible = constraints.isFeasible(
            state, BicycleControl{0.0, 0.0}
        );

        allStatesFeasible &= feasible;

        maxSpeed = std::max(maxSpeed, state.speed);

        maxLateralError = std::max(
            maxLateralError,
            std::abs(state.y - references[k].y)
        );

        std::cout << "Step " << k + 1
                  << " | x=" << state.x
                  << " | y=" << state.y
                  << " | speed=" << state.speed
                  << " | feasible=" << feasible
                  << '\n';
    }

    std::cout << "\nMaximum predicted speed: "
              << maxSpeed << " m/s\n";

    std::cout << "Maximum lateral error: "
              << maxLateralError << " m\n";

    std::cout << "Optimisation cost: "
              << result.totalCost << '\n';

    bool passed =
        allStatesFeasible &&
        maxSpeed <= config.maxSpeed + 1e-5 &&
        maxLateralError < 0.01 &&
        std::isfinite(result.totalCost);

    std::cout << (passed
        ? "STEP 292: PASSED\n"
        : "STEP 292: FAILED\n");

    return passed ? 0 : 1;
}
