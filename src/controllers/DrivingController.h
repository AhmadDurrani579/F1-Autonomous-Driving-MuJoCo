#pragma once

#include "simulation/MuJoCoSimulator.h"

#include <string>

class DrivingController {
public:
    virtual ~DrivingController() = default;

    virtual double computeSteering(const VehicleState& state) = 0;

    virtual double computeTargetSpeed(
        const VehicleState& state,
        double maximumSpeed
    ) const = 0;

    virtual double crossTrackError(
        const VehicleState& state
    ) const = 0;

    virtual std::string name() const = 0;
};
