#pragma once

#include "MPCConfig.h"
#include "../models/KinematicBicycleModel.h"

#include <cmath>

class MPCConstraints {
public:
    explicit MPCConstraints(const MPCConfig& config)
        : config_(config) {}

    bool isFeasible(
        const BicycleState& state,
        const BicycleControl& control
    ) const {
        if (!std::isfinite(state.x) ||
            !std::isfinite(state.y) ||
            !std::isfinite(state.heading) ||
            !std::isfinite(state.speed) ||
            !std::isfinite(control.steering) ||
            !std::isfinite(control.acceleration)) {
            return false;
        }

        return
            std::abs(control.steering) <= config_.maxSteering &&
            control.acceleration >= config_.minAcceleration &&
            control.acceleration <= config_.maxAcceleration &&
            state.speed >= 0.0 &&
            state.speed <= config_.maxSpeed;
    }

private:
    MPCConfig config_;
};
