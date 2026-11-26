#pragma once

#include "DrivingController.h"
#include "MPCConfig.h"
#include "MPCOptimizer.h"
#include "MPCTrackReference.h"

#include <limits>
#include <string>

class MPCController final : public DrivingController {
public:
    explicit MPCController(
        const std::string& waypointFile,
        const MPCConfig& config = MPCConfig{}
    );

    // Called from the simulation physics callback.
    // Optimises at intervals of config.dt seconds.
    void update(
        const VehicleState& state,
        double simulationTime,
        double measuredSteering =
            std::numeric_limits<double>::quiet_NaN()
    );

    // Most recent optimised steering and acceleration.
    BicycleControl latestControl() const {
        return latestControl_;
    }

    bool hasValidControl() const {
        return hasValidControl_;
    }

    double measuredSteering() const {
        return measuredSteering_;
    }

    bool hasMeasuredSteering() const {
        return hasMeasuredSteering_;
    }

    // Existing DrivingController interface.
    double computeSteering(
        const VehicleState& state
    ) override;

    double computeTargetSpeed(
        const VehicleState& state,
        double maximumSpeed
    ) const override;

    double crossTrackError(
        const VehicleState& state
    ) const override;

    std::string name() const override {
        return "MPC";
    }

private:
    MPCConfig config_;

    MPCTrackReference trackReference_;
    MPCOptimizer optimizer_;

    BicycleControl latestControl_{0.0, 0.0};

    double measuredSteering_ = 0.0;
    bool hasMeasuredSteering_ = false;

    bool hasValidControl_ = false;

    double lastSolveTime_ =
        -std::numeric_limits<double>::infinity();
};
