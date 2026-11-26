#include "../src/simulation/MuJoCoSimulator.h"

#include <iomanip>
#include <iostream>

int main() {
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Torque/wheel  Final speed(m/s)  Avg accel(m/s^2)\n";

    for (double torque : {50.0, 100.0, 150.0, 200.0}) {
        MuJoCoSimulator simulator("models/formula1_custom.xml");

        simulator.setSteering(0.0);
        simulator.setRearTorque(torque);

        const double initialSpeed =
            simulator.getVehicleState().speed;

        const double dt = simulator.model()->opt.timestep;
        const int steps = static_cast<int>(1.0 / dt);

        for (int i = 0; i < steps; ++i) {
            simulator.step();
        }

        const double finalSpeed =
            simulator.getVehicleState().speed;

        const double elapsed = simulator.getSimulationTime();

        const double acceleration =
            (finalSpeed - initialSpeed) / elapsed;

        std::cout << std::setw(12) << torque
                  << std::setw(18) << finalSpeed
                  << std::setw(20) << acceleration
                  << '\n';
    }

    std::cout << "STEP 257: COMPLETED\n";
}
