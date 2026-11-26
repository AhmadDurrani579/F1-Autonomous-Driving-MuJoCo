#include "../src/controllers/MPCOptimizer.h"
#include "../src/models/KinematicBicycleModel.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iostream>
#include <numeric>
#include <vector>

int main() {
    MPCConfig config;
    MPCOptimizer optimizer(config);
    KinematicBicycleModel model(config.wheelbase);

    constexpr int cycles = 20;
    const double steering =
        std::atan(config.wheelbase / 20.0);

    BicycleState initialState{0.0, 0.0, 0.0, 2.0};

    // Build a curved reference path long enough
    // for every control cycle and its prediction horizon.
    std::vector<BicycleState> path;
    path.push_back(initialState);

    BicycleState refState = initialState;

    for (int i = 0; i < cycles + config.horizon; ++i) {
        refState = model.predict(
            refState,
            BicycleControl{steering, 0.0},
            config.dt
        );

        path.push_back(refState);
    }

    BicycleState vehicle = initialState;

    std::vector<double> durations;
    int missedDeadlines = 0;
    double maximumPositionError = 0.0;

    using Clock = std::chrono::steady_clock;
    const double deadlineMs = config.dt * 1000.0;

    for (int i = 0; i < cycles; ++i) {
        std::vector<MPCReference> references;

        for (int k = 0; k < config.horizon; ++k) {
            const auto& ref = path[i + k + 1];

            references.push_back({
                ref.x, ref.y, ref.heading, ref.speed
            });
        }

        auto start = Clock::now();

        MPCResult result =
            optimizer.solve(vehicle, references);

        auto finish = Clock::now();

        if (!result.success) {
            std::cerr << "STEP 301: FAILED — Solver error\n";
            return 1;
        }

        double elapsedMs =
            std::chrono::duration<double, std::milli>(
                finish - start
            ).count();

        durations.push_back(elapsedMs);

        if (elapsedMs > deadlineMs) {
            ++missedDeadlines;
        }

        // Apply only the first MPC control.
        vehicle = model.predict(
            vehicle, result.control, config.dt
        );

        double positionError = std::hypot(
            vehicle.x - path[i + 1].x,
            vehicle.y - path[i + 1].y
        );

        maximumPositionError = std::max(
            maximumPositionError, positionError
        );

        std::cout << "Cycle " << i + 1
                  << " | time=" << elapsedMs << " ms"
                  << " | steering="
                  << result.control.steering
                  << " | error="
                  << positionError << " m\n";
    }

    std::sort(durations.begin(), durations.end());

    double average = std::accumulate(
        durations.begin(), durations.end(), 0.0
    ) / durations.size();

    double median =
        (durations[9] + durations[10]) / 2.0;

    std::cout << "\n--- Moving MPC Performance ---\n";
    std::cout << "Minimum: "
              << durations.front() << " ms\n";
    std::cout << "Average: "
              << average << " ms\n";
    std::cout << "Median: "
              << median << " ms\n";
    std::cout << "Maximum: "
              << durations.back() << " ms\n";
    std::cout << "Missed deadlines: "
              << missedDeadlines << "/" << cycles << '\n';
    std::cout << "Maximum position error: "
              << maximumPositionError << " m\n";

    std::cout << "STEP 301: PASSED — Benchmark completed\n";
    return 0;
}
