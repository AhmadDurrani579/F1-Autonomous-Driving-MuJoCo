#include "../src/models/KinematicBicycleModel.h"

#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    constexpr double PI = 3.14159265358979323846;

    KinematicBicycleModel model(3.6);

    BicycleState state{20.0, 10.0, 0.0, 10.0};

    // Constant left steering, no acceleration
    BicycleControl control{10.0 * PI / 180.0, 0.0};

    constexpr double dt = 0.1;
    constexpr int steps = 20;

    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Time(s)   X(m)     Y(m)     Heading(deg)   Speed(m/s)\n";

    for (int k = 0; k <= steps; ++k) {
        std::cout
            << std::setw(6) << k * dt << "   "
            << std::setw(7) << state.x << "   "
            << std::setw(7) << state.y << "   "
            << std::setw(12) << state.heading * 180.0 / PI << "   "
            << std::setw(10) << state.speed << '\n';

        if (k < steps) {
            state = model.predict(state, control, dt);
        }
    }

    const double expectedHeading =
        20.0 * (10.0 / 3.6) * std::tan(10.0 * PI / 180.0)
        * dt * 180.0 / PI;

    const bool passed =
        std::abs(state.heading * 180.0 / PI - expectedHeading) < 1e-6 &&
        std::abs(state.speed - 10.0) < 1e-6;

    std::cout << (passed ? "STEP 237: PASSED\n"
                         : "STEP 237: FAILED\n");

    return passed ? 0 : 1;
}
