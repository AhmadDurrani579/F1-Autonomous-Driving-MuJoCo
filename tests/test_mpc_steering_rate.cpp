#include "../src/controllers/MPCOptimizer.h"
#include "../src/models/KinematicBicycleModel.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

int main() {
    MPCConfig config;
    MPCOptimizer optimizer(config);
    KinematicBicycleModel model(config.wheelbase);

    const double maxChange =
        config.maxSteeringRate * config.dt;

    const double referenceSteering =
        std::atan(config.wheelbase / 20.0);

    BicycleState vehicle{0.0, 0.0, 0.0, 2.0};

    // Generate a reference curve covering two
    // consecutive MPC optimisation cycles.
    std::vector<MPCReference> path;

    BicycleState referenceState = vehicle;

    for (int k = 0; k <= config.horizon; ++k) {
        referenceState = model.predict(
            referenceState,
            BicycleControl{referenceSteering, 0.0},
            config.dt
        );

        path.push_back({
            referenceState.x,
            referenceState.y,
            referenceState.heading,
            referenceState.speed
        });
    }

    bool passed = true;
    double maximumObservedChange = 0.0;
    double previousAppliedSteering = 0.0;

    for (int cycle = 0; cycle < 2; ++cycle) {
        std::vector<MPCReference> references(
            path.begin() + cycle,
            path.begin() + cycle + config.horizon
        );

        MPCResult result = optimizer.solve(
            vehicle,
            references,
            previousAppliedSteering
        );

        if (!result.success ||
            result.predictedControls.size() !=
                static_cast<size_t>(config.horizon)) {
            std::cerr << "STEP 329: FAILED — Solver error\n";
            return 1;
        }

        double previous = previousAppliedSteering;

        for (const auto& control : result.predictedControls) {
            const double change =
                std::abs(control.steering - previous);

            maximumObservedChange = std::max(
                maximumObservedChange, change
            );

            if (!std::isfinite(control.steering) ||
                !std::isfinite(control.acceleration) ||
                change > maxChange + 1e-5 ||
                std::abs(control.steering) >
                    config.maxSteering + 1e-5 ||
                control.acceleration <
                    config.minAcceleration - 1e-5 ||
                control.acceleration >
                    config.maxAcceleration + 1e-5) {
                passed = false;
            }

            previous = control.steering;
        }

        std::cout << "Cycle " << cycle + 1
                  << " first steering: "
                  << result.control.steering
                  << " rad\n";

        // Apply only the first optimised command.
        vehicle = model.predict(
            vehicle,
            result.control,
            config.dt
        );

        previousAppliedSteering =
            result.control.steering;
    }

    std::cout << "Maximum allowed steering change: "
              << maxChange << " rad\n";

    std::cout << "Maximum observed steering change: "
              << maximumObservedChange << " rad\n";

    std::cout << "All steering-rate constraints respected: "
              << (passed ? "YES" : "NO") << '\n';

    std::cout << (passed
        ? "STEP 329: PASSED\n"
        : "STEP 329: FAILED\n");

    return passed ? 0 : 1;
}
