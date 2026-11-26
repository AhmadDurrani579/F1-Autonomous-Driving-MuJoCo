#include <casadi/casadi.hpp>

#include <cmath>
#include <iostream>

int main() {
    try {
        using namespace casadi;

        MX x = MX::sym("x");

        MX objective = pow(x - 3.0, 2);

        MXDict problem;
        problem["x"] = x;
        problem["f"] = objective;

        Dict options;
        options["ipopt.print_level"] = 0;
        options["print_time"] = false;

        Function solver = nlpsol(
            "solver",
            "ipopt",
            problem,
            options
        );

        DMDict arguments;
        arguments["x0"] = 0.0;

        DMDict result = solver(arguments);

        double solution =
            static_cast<double>(result.at("x"));

        double cost =
            static_cast<double>(result.at("f"));

        std::cout << "Optimal x: " << solution << '\n';
        std::cout << "Final cost: " << cost << '\n';

        bool passed =
            std::abs(solution - 3.0) < 1e-5 &&
            std::abs(cost) < 1e-8;

        std::cout << (passed
            ? "STEP 283: PASSED\n"
            : "STEP 283: FAILED\n");

        return passed ? 0 : 1;

    } catch (const std::exception& error) {
        std::cerr << "Solver error: "
                  << error.what() << '\n';

        return 1;
    }
}
