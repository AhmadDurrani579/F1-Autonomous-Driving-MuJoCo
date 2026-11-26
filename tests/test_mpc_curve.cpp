#include "../src/controllers/MPCOptimizer.h"
#include "../src/models/KinematicBicycleModel.h"

#include <cmath>
#include <iostream>
#include <vector>

int main() {
    MPCConfig config;
    MPCOptimizer optimizer(config);
    KinematicBicycleModel model(config.wheelbase);

    BicycleState initialState{0.0, 0.0, 0.0, 2.0};

    // Generate a left-hand curve with a 20 m radius.
    const double radius = 20.0;
    const double referenceSteering =
        std::atan(config.wheelbase / radius);

    BicycleState referenceState = initialState;

    std::vector<MPCReference> references;

    for (int k = 0; k < config.horizon; ++k) {
        referenceState = model.predict(
            referenceState,
            BicycleControl{referenceSteering, 0.0},
            config.dt
        );

        references.push_back({
            referenceState.x,
            referenceState.y,
            referenceState.heading,
            referenceState.speed
        });
    }

    MPCResult result = optimizer.solve(
        initialState,
        references
    );

    if (!result.success ||
        result.predictedStates.size() !=
            static_cast<size_t>(config.horizon)) {
        std::cerr << "STEP 293: FAILED — Solver unsuccessful\n";
        return 1;
    }

    const BicycleState& finalState =
        result.predictedStates.back();

    const MPCReference& finalReference =
        references.back();

    double finalPositionError = std::hypot(
        finalState.x - finalReference.x,
        finalState.y - finalReference.y
    );

    std::cout << "Reference steering: "
              << referenceSteering << " rad\n";

    std::cout << "Optimised first steering: "
              << result.control.steering << " rad\n";

    std::cout << "Optimised acceleration: "
              << result.control.acceleration << " m/s^2\n";

    std::cout << "Final reference position: ("
              << finalReference.x << ", "
              << finalReference.y << ")\n";

    std::cout << "Final predicted position: ("
              << finalState.x << ", "
              << finalState.y << ")\n";

    std::cout << "Final position error: "
              << finalPositionError << " m\n";

    std::cout << "Total cost: "
              << result.totalCost << '\n';

    bool passed =
        std::isfinite(result.totalCost) &&
        result.control.steering > 0.02 &&
        result.control.steering <= config.maxSteering + 1e-5 &&
        std::abs(result.control.acceleration) <=
            config.maxAcceleration + 1e-5 &&
        finalState.y > 0.0 &&
        finalPositionError < 0.2;

    std::cout << (passed
        ? "STEP 293: PASSED\n"
        : "STEP 293: FAILED\n");

    return passed ? 0 : 1;
}
