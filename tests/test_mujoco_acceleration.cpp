#include "../src/simulation/MuJoCoSimulator.h"

#include <iomanip>
#include <iostream>

int main() {
    MuJoCoSimulator simulator("models/formula1_custom.xml");

    simulator.setSteering(0.0);
    simulator.setRearTorque(100.0);

    const double dt = simulator.model()->opt.timestep;
    const int steps = static_cast<int>(2.0 / dt);

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Time(s)  Speed(m/s)  Acceleration(m/s^2)\n";

    double previousSpeed = simulator.getVehicleState().speed;

    for (int i = 1; i <= steps; ++i) {
        simulator.step();

        if (i % 50 == 0) {
            const double speed = simulator.getVehicleState().speed;
            const double acceleration =
                (speed - previousSpeed) / (50.0 * dt);

            std::cout << std::setw(7)
                      << simulator.getSimulationTime()
                      << "  " << std::setw(10) << speed
                      << "  " << std::setw(18) << acceleration
                      << '\n';

            previousSpeed = speed;
        }
    }
}
