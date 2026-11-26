#include "../src/controllers/MPCTrackReference.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <vector>

struct CSVPoint {
    double x, y, heading, s;
};

int main() {
    const std::string trackFile =
        "tracks/custom_waypoints.csv";

    std::ifstream file(trackFile);
    std::string line;
    std::getline(file, line); // Header

    std::vector<CSVPoint> points;

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::replace(line.begin(), line.end(), ',', ' ');

        std::stringstream ss(line);
        CSVPoint p{};

        if (!(ss >> p.x >> p.y >> p.heading >> p.s)) {
            throw std::runtime_error("Invalid CSV row");
        }

        points.push_back(p);
    }

    if (points.size() < 3) {
        std::cerr << "STEP 308: FAILED — Invalid track\n";
        return 1;
    }

    MPCConfig config;
    MPCTrackReference generator(trackFile);

    const auto& first = points.front();
    const auto& last = points.back();

    const double closingDistance = std::hypot(
        first.x - last.x,
        first.y - last.y
    );

    const double targetSpeed = 2.0;
    const double stepDistance =
        targetSpeed * config.dt;

    if (closingDistance <= stepDistance) {
        std::cerr
            << "STEP 308: FAILED — Closing segment too short "
            << "for this test\n";
        return 1;
    }

    // Position the chassis exactly one reference
    // step before the start/finish boundary.
    const double t =
        1.0 - stepDistance / closingDistance;

    const double chassisX =
        last.x + t * (first.x - last.x);

    const double chassisY =
        last.y + t * (first.y - last.y);

    const double headingDifference = std::atan2(
        std::sin(first.heading - last.heading),
        std::cos(first.heading - last.heading)
    );

    const double heading =
        last.heading + t * headingDifference;

    // Convert chassis position to rear-axle state.
    BicycleState initialState{
        chassisX - 1.8 * std::cos(heading),
        chassisY - 1.8 * std::sin(heading),
        heading,
        targetSpeed
    };

    const auto references =
        generator.generate(initialState, config, targetSpeed);

    if (references.size() !=
        static_cast<std::size_t>(config.horizon)) {
        std::cerr << "STEP 308: FAILED — Reference count\n";
        return 1;
    }

    double maxPositionStep = 0.0;
    double maxHeadingStep = 0.0;
    bool finite = true;

    double previousX = 0.0;
    double previousY = 0.0;
    double previousHeading = 0.0;

    for (std::size_t i = 0; i < references.size(); ++i) {
        const auto& ref = references[i];

        // Recover the chassis-centre reference.
        const double x =
            ref.x + 1.8 * std::cos(ref.heading);

        const double y =
            ref.y + 1.8 * std::sin(ref.heading);

        finite &= std::isfinite(x) &&
                  std::isfinite(y) &&
                  std::isfinite(ref.heading) &&
                  std::isfinite(ref.speed);

        if (i > 0) {
            maxPositionStep = std::max(
                maxPositionStep,
                std::hypot(
                    x - previousX,
                    y - previousY
                )
            );

            maxHeadingStep = std::max(
                maxHeadingStep,
                std::abs(std::atan2(
                    std::sin(ref.heading - previousHeading),
                    std::cos(ref.heading - previousHeading)
                ))
            );
        }

        previousX = x;
        previousY = y;
        previousHeading = ref.heading;
    }

    // First reference should reach the first waypoint.
    const auto& firstReference = references.front();

    const double firstRefX =
        firstReference.x +
        1.8 * std::cos(firstReference.heading);

    const double firstRefY =
        firstReference.y +
        1.8 * std::sin(firstReference.heading);

    const double boundaryPositionError = std::hypot(
        firstRefX - first.x,
        firstRefY - first.y
    );

    const double boundaryHeadingError = std::abs(
        std::atan2(
            std::sin(firstReference.heading - first.heading),
            std::cos(firstReference.heading - first.heading)
        )
    );

    std::cout << "Closing segment length: "
              << closingDistance << " m\n";

    std::cout << "Reference count: "
              << references.size() << '\n';

    std::cout << "Start/finish position error: "
              << boundaryPositionError << " m\n";

    std::cout << "Start/finish heading error: "
              << boundaryHeadingError << " rad\n";

    std::cout << "Maximum reference position step: "
              << maxPositionStep << " m\n";

    std::cout << "Maximum reference heading step: "
              << maxHeadingStep << " rad\n";

    bool passed =
        finite &&
        boundaryPositionError < 0.03 &&
        boundaryHeadingError < 0.05 &&
        maxPositionStep < stepDistance * 1.25 &&
        maxHeadingStep < 0.25;

    std::cout << (passed
        ? "STEP 308: PASSED\n"
        : "STEP 308: FAILED\n");

    return passed ? 0 : 1;
}
