#include "MPCOptimizer.h"
#include "MPCConstraints.h"

#include <casadi/casadi.hpp>

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>

MPCOptimizer::MPCOptimizer(const MPCConfig& config)
    : config_(config) {

    if (config_.horizon <= 0 ||
        config_.dt <= 0.0 ||
        config_.wheelbase <= 0.0 ||
        config_.maxSteering <= 0.0 ||
        config_.maxSpeed <= 0.0 ||
        config_.minAcceleration > config_.maxAcceleration) {
        throw std::invalid_argument(
            "Invalid MPC configuration"
        );
    }

    using namespace casadi;

    const int N = config_.horizon;

    // Two controls per prediction step:
    // steering, acceleration.
    MX U = MX::sym("U", 2 * N);

    // Parameters:
    // 4 initial state values + 4 values per reference.
    MX P = MX::sym("P", 6 + 4 * N);

    MX x = P(0);
    MX y = P(1);
    MX psi = P(2);
    MX v = P(3);

    MX physicalSteering = P(5 + 4 * N);

    const double steeringAlpha =
        1.0 - std::exp(
            -config_.dt / config_.steeringResponseTime
        );

    MX totalCost = 0;

    std::vector<MX> speedConstraints;
    speedConstraints.reserve(N);

    for (int k = 0; k < N; ++k) {
        MX steering = U(2 * k);
        MX acceleration = U(2 * k + 1);

        MX priorSteering = (k == 0)
            ? P(4 + 4 * N)
            : U(2 * (k - 1));

        MX steeringChange = steering - priorSteering;

        MX yawSteering =
            config_.enableSteeringDynamics
                ? physicalSteering
                : steering;

        // Predict next state with the bicycle model.
        MX nextX =
            x + v * cos(psi) * config_.dt;

        MX nextY =
            y + v * sin(psi) * config_.dt;

        MX nextPsi =
            psi +
            (v / config_.wheelbase) *
            tan(yawSteering) * config_.dt;

        MX nextV =
            v + acceleration * config_.dt;

        x = nextX;
        y = nextY;
        psi = nextPsi;
        v = nextV;

        if (config_.enableSteeringDynamics) {
            physicalSteering +=
                steeringAlpha *
                (steering - physicalSteering);
        }

        // Four reference values for this step.
        const int offset = 4 + 4 * k;

        MX referenceX = P(offset);
        MX referenceY = P(offset + 1);
        MX referenceHeading = P(offset + 2);
        MX referenceSpeed = P(offset + 3);

        MX dx = x - referenceX;
        MX dy = y - referenceY;

        MX headingDifference =
            psi - referenceHeading;

        MX headingError = atan2(
            sin(headingDifference),
            cos(headingDifference)
        );

        MX speedError = v - referenceSpeed;

        // Same weights as our MPCCost implementation.
        totalCost +=
            config_.positionWeight *
                (pow(dx, 2) + pow(dy, 2)) +
            config_.headingWeight *
                pow(headingError, 2) +
            config_.speedWeight *
                pow(speedError, 2) +
            config_.steeringWeight *
                pow(steering, 2) +
            config_.accelerationWeight *
                pow(acceleration, 2) +
              config_.steeringSmoothnessWeight *
                  pow(steeringChange, 2);

        speedConstraints.push_back(v);
    }

    MXDict problem;

    problem["x"] = U;
    problem["p"] = P;
    problem["f"] = totalCost;
    problem["g"] = vertcat(speedConstraints);

    Dict options;
    options["ipopt.print_level"] = 0;
    options["ipopt.max_iter"] = 200;
    options["print_time"] = false;

    // Create the solver ONCE.
    solver_ = nlpsol(
        "f1_mpc",
        "ipopt",
        problem,
        options
    );
}

MPCResult MPCOptimizer::solve(
    const BicycleState& initialState,
    const std::vector<MPCReference>& references,
    double previousSteering,
    double measuredSteering
) {
    MPCResult result;

    const int N = config_.horizon;

    if (!std::isfinite(measuredSteering)) {
        measuredSteering = previousSteering;
    }

    if (!std::isfinite(previousSteering)) {
        std::cerr << "Invalid previous steering\n";
        return result;
    }

    if (references.size() != static_cast<size_t>(N)) {
        std::cerr << "MPC requires exactly "
                  << N << " reference points\n";
        return result;
    }

    MPCConstraints constraints(config_);

    if (!constraints.isFeasible(
            initialState,
            BicycleControl{0.0, 0.0})) {
        std::cerr << "Invalid initial MPC state\n";
        return result;
    }

    for (const auto& ref : references) {
        if (!std::isfinite(ref.x) ||
            !std::isfinite(ref.y) ||
            !std::isfinite(ref.heading) ||
            !std::isfinite(ref.speed)) {
            std::cerr << "Invalid MPC reference\n";
            return result;
        }
    }

    try {
        using namespace casadi;

        // Current vehicle state + reference trajectory.
        std::vector<double> parameters;
        parameters.reserve(6 + 4 * N);

        parameters.push_back(initialState.x);
        parameters.push_back(initialState.y);
        parameters.push_back(initialState.heading);
        parameters.push_back(initialState.speed);

        for (const auto& ref : references) {
            parameters.push_back(ref.x);
            parameters.push_back(ref.y);
            parameters.push_back(ref.heading);
            parameters.push_back(ref.speed);
        }

        // Previous applied steering for smoothness cost.
        parameters.push_back(previousSteering);
        parameters.push_back(measuredSteering);

        // Bounds on steering and acceleration.
        std::vector<double> lowerControls(2 * N);
        std::vector<double> upperControls(2 * N);
        std::vector<double> initialGuess(2 * N, 0.0);

        for (int k = 0; k < N; ++k) {
            lowerControls[2 * k] =
                -config_.maxSteering;

            upperControls[2 * k] =
                config_.maxSteering;

            lowerControls[2 * k + 1] =
                config_.minAcceleration;

            upperControls[2 * k + 1] =
                config_.maxAcceleration;
        }

        // Shift the previous solution forward one step.
        // This gives IPOPT a useful initial guess.
        if (previousControls_.size() ==
            static_cast<size_t>(2 * N)) {

            for (int k = 0; k < N; ++k) {
                int sourceStep = (k + 1 < N) ? k + 1 : N - 1;

                initialGuess[2 * k] =
                    previousControls_[2 * sourceStep];

                initialGuess[2 * k + 1] =
                    previousControls_[2 * sourceStep + 1];
            }
        }

        DMDict arguments;

        arguments["x0"] = DM(initialGuess);
        arguments["p"] = DM(parameters);

        arguments["lbx"] = DM(lowerControls);
        arguments["ubx"] = DM(upperControls);

        arguments["lbg"] = DM::zeros(N, 1);

        arguments["ubg"] =
            DM::ones(N, 1) * config_.maxSpeed;

        // Reuse the solver created in the constructor.
        DMDict solution = solver_(arguments);

        auto stats = solver_.stats();

        if (!static_cast<bool>(stats.at("success"))) {
            std::cerr << "MPC solver did not converge\n";
            return result;
        }

        DM optimizedControls = solution.at("x");

        KinematicBicycleModel model(config_.wheelbase);

        BicycleState state = initialState;

        double predictedSteering = measuredSteering;

        const double steeringAlpha =
            1.0 - std::exp(
                -config_.dt / config_.steeringResponseTime
            );

        result.predictedStates.reserve(N);
        result.predictedControls.reserve(N);

        std::vector<double> newControls;
        newControls.reserve(2 * N);

        for (int k = 0; k < N; ++k) {
            BicycleControl control{
                static_cast<double>(
                    optimizedControls(2 * k)),
                static_cast<double>(
                    optimizedControls(2 * k + 1))
            };

            newControls.push_back(control.steering);
            newControls.push_back(control.acceleration);

            result.predictedControls.push_back(control);

            BicycleControl physicalControl = control;

            if (config_.enableSteeringDynamics) {
                physicalControl.steering =
                    predictedSteering;
            }

            state = model.predict(
                state,
                physicalControl,
                config_.dt
            );

            if (config_.enableSteeringDynamics) {
                predictedSteering +=
                    steeringAlpha *
                    (control.steering - predictedSteering);
            }

            result.predictedStates.push_back(state);

            if (k == 0) {
                result.control = control;
            }
        }

        result.totalCost =
            static_cast<double>(solution.at("f"));

        result.success = true;

        // Only store a new warm start after success.
        previousControls_ = std::move(newControls);

        return result;

    } catch (const std::exception& error) {
        std::cerr << "MPC optimisation error: "
                  << error.what() << '\n';
        return result;
    }
}
