#include "../src/controllers/MPCCost.h"

#include <cmath>
#include <iostream>

int main() {
    MPCConfig config;
    MPCCost cost(config);

    const MPCReference reference{
        20.0, 10.0, 0.0, 3.0
    };

    const BicycleControl control{
        0.0, 0.0
    };

    // Test 1: Perfect tracking
    const BicycleState perfect{
        20.0, 10.0, 0.0, 3.0
    };

    const double cost1 =
        cost.stageCost(perfect, reference, control);

    // Test 2: One metre position error
    const BicycleState positionError{
        21.0, 10.0, 0.0, 3.0
    };

    const double cost2 =
        cost.stageCost(positionError, reference, control);

    // Test 3: One metre/second speed error
    const BicycleState speedError{
        20.0, 10.0, 0.0, 4.0
    };

    const double cost3 =
        cost.stageCost(speedError, reference, control);

    const bool passed =
        std::abs(cost1 - 0.0) < 1e-9 &&
        std::abs(cost2 - 10.0) < 1e-9 &&
        std::abs(cost3 - 2.0) < 1e-9;

    std::cout << "Perfect tracking cost: "
              << cost1 << '\n';

    std::cout << "Position error cost: "
              << cost2 << '\n';

    std::cout << "Speed error cost: "
              << cost3 << '\n';

    std::cout << (passed
        ? "STEP 266: PASSED\n"
        : "STEP 266: FAILED\n");

    return passed ? 0 : 1;
}
