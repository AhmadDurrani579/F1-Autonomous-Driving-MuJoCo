#include "../src/controllers/MPCOptimizer.h"
#include "../src/controllers/MPCTrackReference.h"
#include "../src/controllers/MPCConstraints.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

int main() {
    const std::string trackFile =
        "tracks/custom_waypoints.csv";

    MPCConfig config;

    MPCTrackReference track(trackFile);
    MPCOptimizer optimizer(config);
    MPCConstraints constraints(config);

    // Read the first real racing-line waypoint.
    std::ifstream file(trackFile);
    std::string line;

    std::getline(file, line); // Skip CSV header
    std::getline(file, line); // First waypoint

    for (char& c : line) {
        if (c == ',') c = ' ';
    }

    std::stringstream ss(line);

    double x, y, heading, s;

    if (!(ss >> x >> y >> heading >> s)) {
        std::cerr << "Invalid starting waypoint\n";
        return 1;
    }

    // The bicycle model represents the rear axle,
    // 1.8 metres behind the chassis centre.
    BicycleState initialState{
        x - 1.8 * std::cos(heading),
        y - 1.8 * std::sin(heading),
        heading,
        2.0
    };

    // Generate 20 references from the real circuit.
    const auto references = track.generate(
        initialState,
        config,
        2.0
    );

    // Optimise steering and acceleration.
    const MPCResult result = optimizer.solve(
        initialState,
        references
    );

    if (!result.success ||
        references.size() !=
            static_cast<size_t>(config.horizon) ||
        result.predictedStates.size() !=
            static_cast<size_t>(config.horizon) ||
        result.predictedControls.size() !=
            static_cast<size_t>(config.horizon)) {

        std::cerr << "STEP 309: FAILED — Invalid MPC result\n";
        return 1;
    }

    bool feasible = true;
    double maxPositionError = 0.0;

    for (int k = 0; k < config.horizon; ++k) {
        const BicycleState& state =
            result.predictedStates[k];

        const BicycleControl& control =
            result.predictedControls[k];

        feasible &= constraints.isFeasible(
            state, control
        );

        const double positionError = std::hypot(
            state.x - references[k].x,
            state.y - references[k].y
        );

        if (!std::isfinite(positionError)) {
            feasible = false;
        }

        maxPositionError = std::max(
            maxPositionError,
            positionError
        );
    }

    const auto& finalState =
        result.predictedStates.back();

    const auto& finalReference =
        references.back();

    const double finalPositionError = std::hypot(
        finalState.x - finalReference.x,
        finalState.y - finalReference.y
    );

    std::cout << "\n--- MPC Real Track Test ---\n";

    std::cout << "Track waypoints: "
              << track.waypointCount() << '\n';

    std::cout << "Reference points: "
              << references.size() << '\n';

    std::cout << "Optimal first steering: "
              << result.control.steering << " rad\n";

    std::cout << "Optimal first acceleration: "
              << result.control.acceleration << " m/s^2\n";

    std::cout << "Final predicted position: ("
              << finalState.x << ", "
              << finalState.y << ")\n";

    std::cout << "Final reference position: ("
              << finalReference.x << ", "
              << finalReference.y << ")\n";

    std::cout << "Maximum position error: "
              << maxPositionError << " m\n";

    std::cout << "Final position error: "
              << finalPositionError << " m\n";

    std::cout << "Total MPC cost: "
              << result.totalCost << '\n';

    std::cout << "All controls and states feasible: "
              << (feasible ? "YES" : "NO") << '\n';

    const bool passed =
        feasible &&
        std::isfinite(result.totalCost) &&
        maxPositionError < 1.0 &&
        finalPositionError < 1.0;

    std::cout << (passed
        ? "STEP 309: PASSED\n"
        : "STEP 309: FAILED\n");

    return passed ? 0 : 1;
}
