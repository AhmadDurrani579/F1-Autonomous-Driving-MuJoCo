#include "MPCController.h"

#include "../models/BicycleStateAdapter.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <stdexcept>

MPCController::MPCController(
    const std::string& waypointFile,
    const MPCConfig& config
)
    : config_(config),
      trackReference_(waypointFile),
      optimizer_(config) {

    std::cout
        << "[MPC] Controller initialised\n"
        << "[MPC] Prediction horizon: "
        << config_.horizon << " steps\n"
        << "[MPC] Control interval: "
        << config_.dt << " seconds\n"
        << "[MPC] Track waypoints: "
        << trackReference_.waypointCount() << '\n';
}

void MPCController::update(
    const VehicleState& state,
    double simulationTime,
    double measuredSteering
) {
    hasMeasuredSteering_ = std::isfinite(measuredSteering);

    if (hasMeasuredSteering_) {
        measuredSteering_ = measuredSteering;
    }

    if (!std::isfinite(simulationTime)) {
        hasValidControl_ = false;
        latestControl_ = {0.0, 0.0};
        return;
    }

    // Detect a simulation time reset.
    if (simulationTime < lastSolveTime_) {
        lastSolveTime_ =
            -std::numeric_limits<double>::infinity();

        hasValidControl_ = false;
        latestControl_ = {0.0, 0.0};
    }

    // Run MPC only once per configured control interval.
    if (simulationTime - lastSolveTime_
        < config_.dt - 1e-9) {
        return;
    }

    // Record the attempted update time even if solving fails,
    // so a failure does not trigger IPOPT every physics step.
    lastSolveTime_ = simulationTime;

    try {
        // Convert chassis-centre state to rear-axle state.
        const BicycleState bicycleState =
            toBicycleState(state);

        // Desired speed for initial MPC integration.
        const double targetSpeed =
            std::min(2.0, config_.maxSpeed);

        // Generate the next 20 racing-line references.
        const auto references = trackReference_.generate(
            bicycleState,
            config_,
            targetSpeed
        );

        // Solve the nonlinear MPC problem.
        const MPCResult result = optimizer_.solve(
              bicycleState,
              references,
              latestControl_.steering,
              hasMeasuredSteering_
                  ? measuredSteering_
                  : latestControl_.steering
          );

        if (!result.success) {
            hasValidControl_ = false;
            latestControl_ = {0.0, 0.0};

            std::cerr
                << "[MPC] Optimisation failed; "
                << "using neutral controls\n";

            return;
        }

        // Apply only the first optimised control.
        // Remaining controls are future predictions.
        // Diagnose abrupt MPC steering changes.
        if (hasValidControl_) {
            const double steeringChange =
                result.control.steering - latestControl_.steering;

            if (std::abs(steeringChange) > 0.03) {
                std::cout
                    << "[MPC STEERING JUMP]"
                    << " time=" << simulationTime
                    << " previous=" << latestControl_.steering
                    << " new=" << result.control.steering
                    << " change=" << steeringChange
                    << std::endl;
            }
        }

        latestControl_ = result.control;
        hasValidControl_ = true;

    } catch (const std::exception& error) {
        hasValidControl_ = false;
        latestControl_ = {0.0, 0.0};

        std::cerr
            << "[MPC] Controller error: "
            << error.what() << '\n';
    }
}

double MPCController::computeSteering(
    const VehicleState& state
) {
    (void)state;

    return hasValidControl_
        ? latestControl_.steering
        : 0.0;
}

double MPCController::computeTargetSpeed(
    const VehicleState& state,
    double maximumSpeed
) const {
    (void)state;

    return std::clamp(
        2.0,
        0.0,
        std::max(0.0, std::min(maximumSpeed, config_.maxSpeed))
    );
}

double MPCController::crossTrackError(
    const VehicleState& state
) const {
    try {
        const BicycleState bicycleState =
            toBicycleState(state);

        // Zero progression returns the nearest track point.
        const auto references = trackReference_.generate(
            bicycleState,
            config_,
            0.0
        );

        if (references.empty()) {
            return 0.0;
        }

        const auto& reference = references.front();

        // Recover chassis-centre track position.
        constexpr double rearAxleOffset = 1.8;

        const double referenceChassisX =
            reference.x +
            rearAxleOffset * std::cos(reference.heading);

        const double referenceChassisY =
            reference.y +
            rearAxleOffset * std::sin(reference.heading);

        return std::hypot(
            state.x - referenceChassisX,
            state.y - referenceChassisY
        );

    } catch (const std::exception&) {
        return 0.0;
    }
}
