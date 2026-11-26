#include "../src/controllers/MPCConstraints.h"

#include <iostream>
#include <limits>

int main() {
    MPCConfig config;
    MPCConstraints constraints(config);

    BicycleState state{20.0, 10.0, 0.0, 2.0};

    struct TestCase {
        const char* name;
        BicycleState state;
        BicycleControl control;
        bool expected;
    };

    const TestCase tests[] = {
        {"Valid control",
         state, {0.2, 1.0}, true},

        {"Steering exceeds limit",
         state, {0.4, 1.0}, false},

        {"Acceleration exceeds limit",
         state, {0.2, 3.0}, false},

        {"Speed exceeds limit",
         {20.0, 10.0, 0.0, 4.0},
         {0.2, 1.0}, false},

        {"Invalid steering value",
         state,
         {std::numeric_limits<double>::quiet_NaN(), 1.0},
         false}
    };

    bool allPassed = true;

    for (const auto& test : tests) {
        const bool actual =
            constraints.isFeasible(test.state, test.control);

        const bool passed = actual == test.expected;

        std::cout << test.name << ": "
                  << (passed ? "PASS" : "FAIL") << '\n';

        allPassed = allPassed && passed;
    }

    std::cout << (allPassed
        ? "STEP 269: PASSED\n"
        : "STEP 269: FAILED\n");

    return allPassed ? 0 : 1;
}
