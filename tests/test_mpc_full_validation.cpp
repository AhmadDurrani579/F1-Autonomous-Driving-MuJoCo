#include "../src/controllers/MPCOptimizer.h"
#include "../src/controllers/MPCConstraints.h"
#include "../src/controllers/MPCCost.h"
#include "../src/models/KinematicBicycleModel.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    MPCConfig config;

    MPCOptimizer optimizer(config);
    MPCConstraints constraints(config);
    MPCCost cost(config);
    KinematicBicycleModel model(config.wheelbase);

    BicycleState initialState{0.0, 0.0, 0.0, 2.0};

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
        initialState, references
    );

    if (!result.success ||
        result.predictedControls.size() !=
            static_cast<size_t>(config.horizon) ||
        result.predictedStates.size() !=
            static_cast<size_t>(config.horizon)) {
        std::cerr << "STEP 295: FAILED — Invalid MPC result\n";
        return 1;
    }

    BicycleState reconstructedState = initialState;

    double recomputedCost = 0.0;
    double maxSteering = 0.0;
    double maxAcceleration = 0.0;
    double maxStateError = 0.0;

    bool allFeasible = true;

    for (int k = 0; k < config.horizon; ++k) {
        const BicycleControl& control =
            result.predictedControls[k];

        reconstructedState = model.predict(
            reconstructedState,
            control,
            config.dt
        );

        const BicycleState& predicted =
            result.predictedStates[k];

        double positionError = std::hypot(
            reconstructedState.x - predicted.x,
            reconstructedState.y - predicted.y
        );

        double headingError = std::abs(
            reconstructedState.heading - predicted.heading
        );

        double speedError = std::abs(
            reconstructedState.speed - predicted.speed
        );

        maxStateError = std::max(
            maxStateError,
            std::max({
                positionError,
                headingError,
                speedError
            })
        );

        allFeasible &=
            constraints.isFeasible(predicted, control);

        maxSteering = std::max(
            maxSteering,
            std::abs(control.steering)
        );

        maxAcceleration = std::max(
            maxAcceleration,
            std::abs(control.acceleration)
        );

        recomputedCost += cost.stageCost(
            reconstructedState,
            references[k],
            control
        );
    }

    const double costDifference =
        std::abs(recomputedCost - result.totalCost);

    std::cout << "Optimised controls: "
              << result.predictedControls.size() << '\n';

    std::cout << "Maximum absolute steering: "
              << maxSteering << " rad\n";

    std::cout << "Maximum absolute acceleration: "
              << maxAcceleration << " m/s^2\n";

    std::cout << "Maximum reconstruction error: "
              << maxStateError << '\n';

    std::cout << "IPOPT cost: "
              << result.totalCost << '\n';

    std::cout << "Recomputed cost: "
              << recomputedCost << '\n';

    std::cout << "Cost difference: "
              << costDifference << '\n';

    bool passed =
        allFeasible &&
        maxStateError < 1e-9 &&
        costDifference < 1e-6 &&
        std::isfinite(recomputedCost) &&
        maxSteering <= config.maxSteering + 1e-5 &&
        maxAcceleration <=
            config.maxAcceleration + 1e-5;

    std::cout << (passed
        ? "STEP 295: PASSED\n"
        : "STEP 295: FAILED\n");

    return passed ? 0 : 1;
}
