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

    BicycleState initialState{0.0, 0.0, 0.0, 2.0};

    const double steering =
        std::atan(config.wheelbase / 20.0);

    BicycleState referenceState = initialState;
    std::vector<MPCReference> references;

    for (int k = 0; k < config.horizon; ++k) {
        referenceState = model.predict(
            referenceState,
            BicycleControl{steering, 0.0},
            config.dt
        );

        references.push_back({
            referenceState.x,
            referenceState.y,
            referenceState.heading,
            referenceState.speed
        });
    }

    constexpr int iterations = 10;
    std::vector<double> durations;

    using Clock = std::chrono::steady_clock;

    for (int i = 0; i < iterations; ++i) {
        auto start = Clock::now();

        MPCResult result = optimizer.solve(
            initialState, references
        );

        auto finish = Clock::now();

        if (!result.success) {
            std::cerr << "STEP 296: FAILED\n";
            return 1;
        }

        double elapsedMs =
            std::chrono::duration<double, std::milli>(
                finish - start
            ).count();

        durations.push_back(elapsedMs);

        std::cout << "Optimisation " << i + 1
                  << ": " << elapsedMs
                  << " ms\n";
    }

    std::sort(durations.begin(), durations.end());

    double average = std::accumulate(
        durations.begin(),
        durations.end(),
        0.0
    ) / durations.size();

    double median =
        (durations[4] + durations[5]) / 2.0;

    double deadline = config.dt * 1000.0;

    int missedDeadlines = std::count_if(
        durations.begin(),
        durations.end(),
        [deadline](double ms) {
            return ms > deadline;
        }
    );

    std::cout << "\n--- MPC Performance ---\n";
    std::cout << "Minimum: " << durations.front() << " ms\n";
    std::cout << "Average: " << average << " ms\n";
    std::cout << "Median: " << median << " ms\n";
    std::cout << "Maximum: " << durations.back() << " ms\n";
    std::cout << "Control deadline: " << deadline << " ms\n";
    std::cout << "Missed deadlines: "
              << missedDeadlines << "/" << iterations << '\n';

    std::cout << "STEP 296: PASSED\n";
    return 0;
}
