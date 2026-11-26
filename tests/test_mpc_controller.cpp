#include "../src/controllers/MPCController.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>

int main() {
    const std::string trackFile =
        "tracks/custom_waypoints.csv";

    MPCConfig config;
    MPCController controller(trackFile, config);

    // Read the first waypoint of our real racing circuit.
    std::ifstream file(trackFile);
    std::string line;

    std::getline(file, line); // Header
    std::getline(file, line); // First waypoint

    std::replace(line.begin(), line.end(), ',', ' ');

    std::stringstream ss(line);

    double x, y, heading, s;

    if (!(ss >> x >> y >> heading >> s)) {
        std::cerr << "STEP 312: FAILED — Invalid waypoint\n";
        return 1;
    }

    VehicleState state;
    state.x = x;
    state.y = y;
    state.heading = heading;
    state.speed = 2.0;

    // First optimisation at simulation time 0.
    controller.update(state, 0.0);

    if (!controller.hasValidControl()) {
        std::cerr << "STEP 312: FAILED — First solve\n";
        return 1;
    }

    const BicycleControl first =
        controller.latestControl();

    std::cout << "First steering: "
              << first.steering << " rad\n";

    std::cout << "First acceleration: "
              << first.acceleration << " m/s^2\n";

    // At 0.05 seconds, MPC must NOT run again.
    // An invalid-speed state is used to detect
    // an unintended early optimisation.
    VehicleState invalidState = state;
    invalidState.speed = config.maxSpeed + 1.0;

    controller.update(invalidState, 0.05);

    const BicycleControl held =
        controller.latestControl();

    const bool commandHeld =
        controller.hasValidControl() &&
        held.steering == first.steering &&
        held.acceleration == first.acceleration;

    std::cout << "Control held at 0.05 s: "
              << (commandHeld ? "YES" : "NO") << '\n';

    // At 0.1 seconds, another solve is permitted.
    controller.update(state, 0.1);

    const BicycleControl second =
        controller.latestControl();

    const bool secondSolveValid =
        controller.hasValidControl() &&
        std::isfinite(second.steering) &&
        std::isfinite(second.acceleration);

    std::cout << "Control valid at 0.1 s: "
              << (secondSolveValid ? "YES" : "NO") << '\n';

    std::cout << "Updated steering: "
              << second.steering << " rad\n";

    std::cout << "Updated acceleration: "
              << second.acceleration << " m/s^2\n";

    const double cte = controller.crossTrackError(state);

    std::cout << "Initial cross-track error: "
              << cte << " m\n";

    const bool withinLimits =
        std::abs(second.steering) <=
            config.maxSteering + 1e-5 &&
        second.acceleration >=
            config.minAcceleration - 1e-5 &&
        second.acceleration <=
            config.maxAcceleration + 1e-5;

    const bool passed =
        commandHeld &&
        secondSolveValid &&
        withinLimits &&
        std::isfinite(cte) &&
        cte < 0.05;

    std::cout << (passed
        ? "STEP 312: PASSED\n"
        : "STEP 312: FAILED\n");

    return passed ? 0 : 1;
}
