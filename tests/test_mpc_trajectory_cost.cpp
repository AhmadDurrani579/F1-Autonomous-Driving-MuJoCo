#include "../src/controllers/MPCCost.h"
#include "../src/models/KinematicBicycleModel.h"

#include <cmath>
#include <iostream>

int main() {
    MPCConfig config;
    MPCCost cost(config);
    KinematicBicycleModel model(config.wheelbase);

    BicycleState state{20.0, 10.0, 0.0, 3.0};
    BicycleControl control{0.0, 0.0};

    double perfectCost = 0.0;
    double offsetCost = 0.0;

    for (int k = 0; k < config.horizon; ++k) {
        // Predict the next state.
        state = model.predict(state, control, config.dt);

        // Reference at the same future time.
        MPCReference reference{
            20.0 + (k + 1) * 3.0 * config.dt,
            10.0,
            0.0,
            3.0
        };

        perfectCost += cost.stageCost(
            state, reference, control
        );

        // Reference shifted 1 metre sideways.
        reference.y = 11.0;

        offsetCost += cost.stageCost(
            state, reference, control
        );
    }

    const bool passed =
        std::abs(perfectCost) < 1e-8 &&
        std::abs(offsetCost - 200.0) < 1e-8;

    std::cout << "Perfect trajectory cost: "
              << perfectCost << '\n';

    std::cout << "Offset trajectory cost: "
              << offsetCost << '\n';

    std::cout << (passed
        ? "STEP 267: PASSED\n"
        : "STEP 267: FAILED\n");

    return passed ? 0 : 1;
}
