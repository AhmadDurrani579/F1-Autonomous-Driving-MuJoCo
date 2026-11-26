#pragma once

#include "MPCConfig.h"
#include "../models/KinematicBicycleModel.h"

#include <cmath>

struct MPCReference {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double speed = 0.0;
};

class MPCCost {
public:
    explicit MPCCost(const MPCConfig& config)
        : config_(config) {}

    double stageCost(
        const BicycleState& state,
        const MPCReference& reference,
        const BicycleControl& control
    ) const {
        const double dx = state.x - reference.x;
        const double dy = state.y - reference.y;

        const double headingDifference =
            state.heading - reference.heading;

        const double headingError =
            std::atan2(
                std::sin(headingDifference),
                std::cos(headingDifference)
            );

        const double speedError =
            state.speed - reference.speed;

        return
            config_.positionWeight * (dx * dx + dy * dy)
            + config_.headingWeight * headingError * headingError
            + config_.speedWeight * speedError * speedError
            + config_.steeringWeight
                * control.steering * control.steering
            + config_.accelerationWeight
                * control.acceleration * control.acceleration;
    }

private:
    MPCConfig config_;
};
