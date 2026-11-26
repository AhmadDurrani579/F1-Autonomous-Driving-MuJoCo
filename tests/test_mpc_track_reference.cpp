#include "../src/controllers/MPCTrackReference.h"

#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <vector>

struct CSVPoint {
    double x, y, heading, s;
};

CSVPoint readPoint(const std::string& line) {
    CSVPoint p{};
    char comma;

    std::stringstream ss(line);

    ss >> p.x >> comma
       >> p.y >> comma
       >> p.heading >> comma
       >> p.s;

    return p;
}

int main() {
    const std::string trackFile =
        "tracks/custom_waypoints.csv";

    MPCConfig config;
    MPCTrackReference generator(trackFile);

    std::ifstream file(trackFile);

    std::string line;
    std::getline(file, line); // Header

    std::vector<CSVPoint> points;

    while (std::getline(file, line)) {
        if (!line.empty()) {
            points.push_back(readPoint(line));
        }
    }

    if (points.size() < 3) {
        std::cerr << "STEP 307: FAILED — Invalid track\n";
        return 1;
    }

    // Place the car's rear axle at the first
    // waypoint's corresponding rear-axle position.
    const CSVPoint& first = points.front();

    BicycleState initialState{
        first.x - 1.8 * std::cos(first.heading),
        first.y - 1.8 * std::sin(first.heading),
        first.heading,
        2.0
    };

    auto references = generator.generate(
        initialState, config, 2.0
    );

    bool valid =
        generator.waypointCount() == points.size() &&
        references.size() ==
            static_cast<std::size_t>(config.horizon);

    // Independently calculate the first interpolated
    // chassis reference at s = 0.2 metres.
    const CSVPoint& second = points[1];

    const double alpha =
        (2.0 * config.dt) / (second.s - first.s);

    const double expectedX =
        first.x + alpha * (second.x - first.x);

    const double expectedY =
        first.y + alpha * (second.y - first.y);

    const auto& ref = references.front();

    const double actualChassisX =
        ref.x + 1.8 * std::cos(ref.heading);

    const double actualChassisY =
        ref.y + 1.8 * std::sin(ref.heading);

    const double interpolationError = std::hypot(
        actualChassisX - expectedX,
        actualChassisY - expectedY
    );

    valid &= interpolationError < 1e-6;

    for (const auto& reference : references) {
        valid &= std::isfinite(reference.x);
        valid &= std::isfinite(reference.y);
        valid &= std::isfinite(reference.heading);
        valid &= std::isfinite(reference.speed);

        valid &= std::abs(reference.speed - 2.0) < 1e-9;
    }

    // Test reference generation near the circuit end.
    const CSVPoint& last = points.back();

    BicycleState endState{
        last.x - 1.8 * std::cos(last.heading),
        last.y - 1.8 * std::sin(last.heading),
        last.heading,
        2.0
    };

    auto endReferences = generator.generate(
        endState, config, 2.0
    );

    valid &= endReferences.size() ==
        static_cast<std::size_t>(config.horizon);

    for (const auto& reference : endReferences) {
        valid &= std::isfinite(reference.x);
        valid &= std::isfinite(reference.y);
        valid &= std::isfinite(reference.heading);
        valid &= std::isfinite(reference.speed);
    }

    std::cout << "Track waypoints: "
              << generator.waypointCount() << '\n';

    std::cout << "Generated references: "
              << references.size() << '\n';

    std::cout << "First reference position: ("
              << ref.x << ", " << ref.y << ")\n";

    std::cout << "First reference heading: "
              << ref.heading << " rad\n";

    std::cout << "First reference speed: "
              << ref.speed << " m/s\n";

    std::cout << "Interpolation error: "
              << interpolationError << " m\n";

    std::cout << "End-of-track references: "
              << endReferences.size() << '\n';

    std::cout << (valid
        ? "STEP 307: PASSED\n"
        : "STEP 307: FAILED\n");

    return valid ? 0 : 1;
}
