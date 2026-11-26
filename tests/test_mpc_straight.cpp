#include "../src/controllers/MPCOptimizer.h"

#include <cmath>
#include <iostream>
#include <vector>

int main() {
    MPCConfig config;
    MPCOptimizer optimizer(config);

    BicycleState initialState{0.0, 0.0, 0.0, 1.0};

    std::vector<MPCReference> references;

    for (int k = 0; k < config.horizon; ++k) {
        double t = (k + 1) * config.dt;

        references.push_back({
            2.0 * t,
            0.0,
            0.0,
            2.0
        });
    }

    MPCResult result = optimizer.solve(
        initialState,
        references
    );

    if (!result.success) {
        std::cerr << "STEP 291: FAILED — Solver unsuccessful\n";
        return 1;
    }

    std::cout << "Optimal steering: "
              << result.control.steering << " rad\n";

    std::cout << "Optimal acceleration: "
              << result.control.acceleration << " m/s^2\n";

    std::cout << "Total cost: "
              << result.totalCost << '\n';

    std::cout << "Predicted states: "
              << result.predictedStates.size() << '\n';

    const BicycleState& finalState =
        result.predictedStates.back();

    std::cout << "Final predicted position: ("
              << finalState.x << ", "
              << finalState.y << ")\n";

    std::cout << "Final predicted speed: "
              << finalState.speed << " m/s\n";

    bool passed =
        result.predictedStates.size() ==
            static_cast<size_t>(config.horizon) &&
        std::abs(result.control.steering) < 0.01 &&
        result.control.acceleration > 0.0 &&
        result.control.acceleration <=
            config.maxAcceleration + 1e-5 &&
        std::isfinite(result.totalCost) &&
        finalState.speed > initialState.speed &&
        finalState.speed <= config.maxSpeed + 1e-5;

    std::cout << (passed
        ? "STEP 291: PASSED\n"
        : "STEP 291: FAILED\n");

    return passed ? 0 : 1;
}
