#include "../src/models/BicycleStateAdapter.h"

#include <cmath>
#include <iostream>

int main() {
    constexpr double PI = 3.14159265358979323846;

    VehicleState vehicle;
    vehicle.x = 20.0;
    vehicle.y = 10.0;
    vehicle.heading = 0.0;
    vehicle.speed = 10.0;

    BicycleState rear = toBicycleState(vehicle);

    bool test1 =
        std::abs(rear.x - 18.2) < 1e-6 &&
        std::abs(rear.y - 10.0) < 1e-6;

    std::cout << "Test 1 (0 degrees): "
              << rear.x << ", " << rear.y << '\n';

    vehicle.heading = PI / 2.0;
    rear = toBicycleState(vehicle);

    bool test2 =
        std::abs(rear.x - 20.0) < 1e-6 &&
        std::abs(rear.y - 8.2) < 1e-6;

    std::cout << "Test 2 (90 degrees): "
              << rear.x << ", " << rear.y << '\n';

    if (test1 && test2) {
        std::cout << "STEP 246: PASSED\n";
        return 0;
    }

    std::cout << "STEP 246: FAILED\n";
    return 1;
}
