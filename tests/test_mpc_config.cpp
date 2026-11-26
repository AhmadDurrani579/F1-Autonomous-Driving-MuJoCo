#include "../src/controllers/MPCConfig.h"

#include <cmath>
#include <iostream>

int main() {
    MPCConfig config;

    const double predictionDuration =
        config.horizon * config.dt;

    const double maximumTorque =
        config.maxAcceleration * 140.0;

    const bool passed =
        config.horizon == 20 &&
        std::abs(predictionDuration - 2.0) < 1e-9 &&
        std::abs(config.wheelbase - 3.6) < 1e-9 &&
        config.maxSteering > 0.0 &&
        config.maxSteering <= 0.5 &&
        config.minAcceleration < 0.0 &&
        config.maxAcceleration > 0.0 &&
        maximumTorque <= 500.0 &&
        config.maxSpeed > 0.0;

    std::cout << "Prediction horizon: "
              << config.horizon << " steps\n";

    std::cout << "Prediction duration: "
              << predictionDuration << " seconds\n";

    std::cout << "Wheelbase: "
              << config.wheelbase << " metres\n";

    std::cout << "Maximum steering: "
              << config.maxSteering * 180.0 / std::acos(-1.0)
              << " degrees\n";

    std::cout << "Maximum acceleration torque: "
              << maximumTorque << " per rear wheel\n";

    std::cout << (passed
        ? "STEP 264: PASSED\n"
        : "STEP 264: FAILED\n");

    return passed ? 0 : 1;
}
