#include "../src/simulation/MuJoCoSimulator.h"
#include "../src/models/KinematicBicycleModel.h"
#include "../src/models/BicycleStateAdapter.h"

#include <cmath>
#include <iomanip>
#include <iostream>

int main() {
    MuJoCoSimulator simulator("models/formula1_custom.xml");
    KinematicBicycleModel bicycle(3.6);

    // Read initial chassis state and convert to rear axle.
    VehicleState initial = simulator.getVehicleState();
    BicycleState predicted = toBicycleState(initial);

    // First experiment: no steering or drive torque.
    simulator.setSteering(0.0);
    simulator.setRearTorque(0.0);

    constexpr int physicsSteps = 50;
    constexpr double dt = 0.002;

    // Predict 0.1 seconds using the bicycle model.
    for (int i = 0; i < physicsSteps; ++i) {
        predicted = bicycle.predict(
            predicted,
            BicycleControl{0.0, 0.0},
            dt
        );
    }

    // Advance MuJoCo by exactly 0.1 seconds.
    for (int i = 0; i < physicsSteps; ++i) {
        simulator.step();
    }

    BicycleState actual =
        toBicycleState(simulator.getVehicleState());

    double positionError = std::hypot(
        actual.x - predicted.x,
        actual.y - predicted.y
    );

    double headingError = std::atan2(
        std::sin(actual.heading - predicted.heading),
        std::cos(actual.heading - predicted.heading)
    );

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Simulation time: "
              << simulator.getSimulationTime() << " s\n";
    std::cout << "Predicted X/Y: "
              << predicted.x << ", " << predicted.y << '\n';
    std::cout << "MuJoCo X/Y:    "
              << actual.x << ", " << actual.y << '\n';
    std::cout << "Position error: " << positionError << " m\n";
    std::cout << "Heading error: "
              << headingError * 180.0 / std::acos(-1.0)
              << " degrees\n";

    return 0;
}
