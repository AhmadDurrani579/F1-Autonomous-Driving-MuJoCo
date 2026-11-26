#include "../src/controllers/MPCOptimizer.h"
#include "../src/models/KinematicBicycleModel.h"

#include <cmath>
#include <iostream>
#include <vector>

double steeringVariation(
    const MPCResult& result,
    double previousSteering
) {
    double variation = 0.0;

    for (const auto& control : result.predictedControls) {
        const double change =
            control.steering - previousSteering;

        variation += change * change;
        previousSteering = control.steering;
    }

    return variation;
}

int main() {
    MPCConfig originalConfig;
    originalConfig.steeringSmoothnessWeight = 0.0;

    MPCConfig smoothConfig;
    smoothConfig.steeringSmoothnessWeight = 10.0;

    MPCOptimizer originalMPC(originalConfig);
    MPCOptimizer smoothMPC(smoothConfig);

    KinematicBicycleModel model(smoothConfig.wheelbase);

    BicycleState initialState{0.0, 0.0, 0.0, 2.0};
    BicycleState referenceState = initialState;

    std::vector<MPCReference> references;

    // Generate a curved reference trajectory.
    for (int k = 0; k < smoothConfig.horizon; ++k) {
        referenceState = model.predict(
            referenceState,
            BicycleControl{0.18, 0.0},
            smoothConfig.dt
        );

        references.push_back({
            referenceState.x,
            referenceState.y,
            referenceState.heading,
            referenceState.speed
        });
    }

    // Previous steering deliberately differs from
    // the steering needed for the reference curve.
    const double previousSteering = -0.18;

    MPCResult original = originalMPC.solve(
        initialState, references, previousSteering
    );

    MPCResult smooth = smoothMPC.solve(
        initialState, references, previousSteering
    );

    if (!original.success || !smooth.success) {
        std::cerr << "STEP 337: FAILED — Solver error\n";
        return 1;
    }

    if (original.predictedControls.size() !=
            static_cast<size_t>(smoothConfig.horizon) ||
        smooth.predictedControls.size() !=
            static_cast<size_t>(smoothConfig.horizon)) {
        std::cerr << "STEP 337: FAILED — Invalid prediction size\n";
        return 1;
    }

    const double originalVariation =
        steeringVariation(original, previousSteering);

    const double smoothVariation =
        steeringVariation(smooth, previousSteering);

    std::cout << "\n=== MPC STEERING COMPARISON ===\n";

    std::cout << "Original first steering: "
              << original.control.steering << " rad\n";

    std::cout << "Smooth first steering: "
              << smooth.control.steering << " rad\n";

    std::cout << "Original steering variation: "
              << originalVariation << '\n';

    std::cout << "Improved steering variation: "
              << smoothVariation << '\n';

    std::cout << "Original optimisation cost: "
              << original.totalCost << '\n';

    std::cout << "Improved optimisation cost: "
              << smooth.totalCost << '\n';

    const bool passed =
        std::isfinite(originalVariation) &&
        std::isfinite(smoothVariation) &&
        smoothVariation < originalVariation;

    std::cout << "\nSteering variation reduced: "
              << (passed ? "YES" : "NO") << '\n';

    std::cout << (passed
        ? "STEP 337: PASSED\n"
        : "STEP 337: FAILED\n");

    return passed ? 0 : 1;
}
