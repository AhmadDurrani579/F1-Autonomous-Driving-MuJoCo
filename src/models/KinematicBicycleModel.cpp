#include "KinematicBicycleModel.h"

#include <cmath>
#include <stdexcept>

KinematicBicycleModel::KinematicBicycleModel(double wheelbase)
    : wheelbase_(wheelbase) {
    if (wheelbase <= 0.0) {
        throw std::invalid_argument("Wheelbase must be positive");
    }
}

BicycleState KinematicBicycleModel::predict(
    const BicycleState& state,
    const BicycleControl& control,
    double dt
) const {
    if (dt <= 0.0) {
        throw std::invalid_argument("Time step must be positive");
    }

    BicycleState next;

    // Equation 1: Predict X position
    next.x = state.x
           + state.speed * std::cos(state.heading) * dt;

    // Equation 2: Predict Y position
    next.y = state.y
           + state.speed * std::sin(state.heading) * dt;

    // Equation 3: Predict heading (yaw)
    next.heading = state.heading
                 + (state.speed / wheelbase_)
                 * std::tan(control.steering) * dt;

    // Equation 4: Predict speed
    next.speed = state.speed + control.acceleration * dt;

    return next;
}
