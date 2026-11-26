#include "../src/models/KinematicBicycleModel.h"

#include <cmath>
#include <iostream>

int main() {
    constexpr double PI = 3.14159265358979323846;

    KinematicBicycleModel model(3.6);

    BicycleState state{
        20.0, 10.0, 0.0, 10.0
    };

    BicycleControl control{
        10.0 * PI / 180.0, 1.0
    };

    BicycleState next = model.predict(state, control, 0.1);

    const double headingDegrees = next.heading * 180.0 / PI;

    std::cout << "Predicted X: " << next.x << " m\n";
    std::cout << "Predicted Y: " << next.y << " m\n";
    std::cout << "Predicted heading: " << headingDegrees << " degrees\n";
    std::cout << "Predicted speed: " << next.speed << " m/s\n";

    const bool passed =
        std::abs(next.x - 21.0) < 1e-6 &&
        std::abs(next.y - 10.0) < 1e-6 &&
        std::abs(headingDegrees - 2.806) < 0.01 &&
        std::abs(next.speed - 10.1) < 1e-6;

    std::cout << (passed ? "STEP 236: PASSED\n"
                         : "STEP 236: FAILED\n");

    return passed ? 0 : 1;
}
