#include "../src/simulation/MuJoCoSimulator.h"

#include <iomanip>
#include <iostream>

int main() {
    MuJoCoSimulator simulator("models/formula1_custom.xml");

    const double dt = simulator.model()->opt.timestep;
    const int stepsPerTenth = static_cast<int>(0.1 / dt);

    simulator.setSteering(0.0);

    // Phase 1: Accelerate for 2 seconds.
    simulator.setRearTorque(200.0);

    for (int i = 0; i < 20 * stepsPerTenth; ++i) {
        simulator.step();
    }

    const double initialSpeed = simulator.getVehicleState().speed;

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Speed before braking: "
              << initialSpeed << " m/s\n";

    // Phase 2: Apply negative torque.
    simulator.setRearTorque(-100.0);

    double previousSpeed = initialSpeed;

    std::cout << "Brake time(s)  Speed(m/s)  Avg accel(m/s^2)\n";

    for (int k = 1; k <= 10; ++k) {
        for (int i = 0; i < stepsPerTenth; ++i) {
            simulator.step();
        }

        const double speed = simulator.getVehicleState().speed;
        const double acceleration =
            (speed - previousSpeed) / 0.1;

        std::cout << std::setw(13) << k * 0.1
                  << std::setw(12) << speed
                  << std::setw(18) << acceleration
                  << '\n';

        previousSpeed = speed;
    }

    std::cout << "STEP 259: COMPLETED\n";
}
