#pragma once

#include "KinematicBicycleModel.h"
#include "../simulation/MuJoCoSimulator.h"

#include <cmath>

inline BicycleState toBicycleState(
    const VehicleState& vehicle,
    double rearAxleOffset = 1.8
) {
    BicycleState state;

    state.x = vehicle.x
            - rearAxleOffset * std::cos(vehicle.heading);

    state.y = vehicle.y
            - rearAxleOffset * std::sin(vehicle.heading);

    state.heading = vehicle.heading;

    // Provisional: chassis speed approximation.
    // Replace with rear-axle longitudinal velocity
    // during model validation.
    state.speed = vehicle.speed;

    return state;
}
