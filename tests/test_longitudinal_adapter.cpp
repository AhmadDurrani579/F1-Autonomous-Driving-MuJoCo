#include "../src/models/LongitudinalControlAdapter.h"

#include <cmath>
#include <iostream>

int main() {
    LongitudinalControlAdapter adapter;

    struct TestCase {
        double acceleration;
        double expectedTorque;
    };

    const TestCase tests[] = {
        { 1.0,  140.0 },
        { 2.0,  280.0 },
        {-1.0, -140.0 },
        { 5.0,  500.0 },
        {-5.0, -500.0 }
    };

    bool allPassed = true;

    for (const auto& test : tests) {
        const double actual =
            adapter.toTorque(test.acceleration);

        const bool passed =
            std::abs(actual - test.expectedTorque) < 1e-9;

        std::cout
            << "Acceleration: " << test.acceleration
            << " m/s^2 | Torque: " << actual
            << " | " << (passed ? "PASS" : "FAIL")
            << '\n';

        allPassed = allPassed && passed;
    }

    std::cout << (allPassed
        ? "STEP 262: PASSED\n"
        : "STEP 262: FAILED\n");

    return allPassed ? 0 : 1;
}
