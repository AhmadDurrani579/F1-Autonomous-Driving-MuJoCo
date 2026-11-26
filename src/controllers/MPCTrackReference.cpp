#include "MPCTrackReference.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

MPCTrackReference::MPCTrackReference(
    const std::string& waypointFile,
    double rearAxleOffset
)
    : rearAxleOffset_(rearAxleOffset) {

    if (!std::isfinite(rearAxleOffset_) ||
        rearAxleOffset_ < 0.0) {
        throw std::invalid_argument(
            "Invalid rear axle offset"
        );
    }

    std::ifstream file(waypointFile);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open MPC waypoint file: " + waypointFile
        );
    }

    std::string line;
    std::getline(file, line); // CSV header

    while (std::getline(file, line)) {
        if (line.empty()) continue;

        std::stringstream ss(line);
        std::string values[4];

        for (int i = 0; i < 4; ++i) {
            if (!std::getline(ss, values[i], ',')) {
                throw std::runtime_error(
                    "Malformed MPC waypoint: " + line
                );
            }
        }

        TrackWaypoint point{
            std::stod(values[0]),
            std::stod(values[1]),
            std::stod(values[2]),
            std::stod(values[3])
        };

        if (!std::isfinite(point.x) ||
            !std::isfinite(point.y) ||
            !std::isfinite(point.heading) ||
            !std::isfinite(point.s)) {
            throw std::runtime_error(
                "Non-finite MPC waypoint"
            );
        }

        waypoints_.push_back(point);
    }

    if (waypoints_.size() < 3) {
        throw std::runtime_error(
            "MPC requires at least three waypoints"
        );
    }

    // Normalise cumulative distance to begin at zero.
    const double initialS = waypoints_.front().s;

    for (auto& point : waypoints_) {
        point.s -= initialS;
    }

    // Cumulative distance must increase along the track.
    for (std::size_t i = 1; i < waypoints_.size(); ++i) {
        if (waypoints_[i].s <= waypoints_[i - 1].s) {
            throw std::runtime_error(
                "MPC waypoint distances must increase"
            );
        }
    }

    // Closing distance from final waypoint to first.
    const auto& first = waypoints_.front();
    const auto& last = waypoints_.back();

    const double closingDistance = std::hypot(
        first.x - last.x,
        first.y - last.y
    );

    trackLength_ = last.s + closingDistance;

    if (trackLength_ <= 0.0) {
        throw std::runtime_error(
            "Invalid MPC track length"
        );
    }
}

std::vector<MPCReference> MPCTrackReference::generate(
    const BicycleState& currentState,
    const MPCConfig& config,
    double targetSpeed
) const {
    if (config.horizon <= 0 ||
        !std::isfinite(config.dt) ||
        config.dt <= 0.0 ||
        !std::isfinite(config.maxSpeed) ||
        config.maxSpeed <= 0.0 ||
        !std::isfinite(targetSpeed) ||
        targetSpeed < 0.0 ||
        targetSpeed > config.maxSpeed ||
        !std::isfinite(currentState.x) ||
        !std::isfinite(currentState.y) ||
        !std::isfinite(currentState.heading)) {
        throw std::invalid_argument(
            "Invalid MPC reference parameters"
        );
    }

    // The track represents the chassis racing line.
    // Convert current rear-axle position to chassis centre.
    const double chassisX =
        currentState.x +
        rearAxleOffset_ * std::cos(currentState.heading);

    const double chassisY =
        currentState.y +
        rearAxleOffset_ * std::sin(currentState.heading);

    // Find the nearest projection onto the closed track.
    double nearestS = 0.0;
    double bestDistanceSquared =
        std::numeric_limits<double>::infinity();

    const std::size_t count = waypoints_.size();

    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t next = (i + 1) % count;

        const auto& a = waypoints_[i];
        const auto& b = waypoints_[next];

        const double dx = b.x - a.x;
        const double dy = b.y - a.y;

        const double lengthSquared = dx * dx + dy * dy;

        if (lengthSquared < 1e-12) continue;

        const double t = std::clamp(
            ((chassisX - a.x) * dx +
             (chassisY - a.y) * dy) / lengthSquared,
            0.0,
            1.0
        );

        const double projectedX = a.x + t * dx;
        const double projectedY = a.y + t * dy;

        const double distanceSquared =
            std::pow(chassisX - projectedX, 2) +
            std::pow(chassisY - projectedY, 2);

        if (distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;

            const double endS =
                (next == 0) ? trackLength_ : b.s;

            nearestS = a.s + t * (endS - a.s);
        }
    }

    if (!std::isfinite(bestDistanceSquared)) {
        throw std::runtime_error(
            "No valid MPC track segment"
        );
    }

    std::vector<MPCReference> references;
    references.reserve(config.horizon);

    for (int k = 0; k < config.horizon; ++k) {
        // Distance travelled along the racing line.
        double referenceS = std::fmod(
            nearestS + (k + 1) * targetSpeed * config.dt,
            trackLength_
        );

        // Find the segment containing referenceS.
        auto upper = std::upper_bound(
            waypoints_.begin(),
            waypoints_.end(),
            referenceS,
            [](double s, const TrackWaypoint& point) {
                return s < point.s;
            }
        );

        std::size_t index;

        if (upper == waypoints_.begin()) {
            index = 0;
        } else {
            index = static_cast<std::size_t>(
                upper - waypoints_.begin() - 1
            );
        }

        const std::size_t next = (index + 1) % count;

        const auto& a = waypoints_[index];
        const auto& b = waypoints_[next];

        const double endS =
            (next == 0) ? trackLength_ : b.s;

        const double segmentLength = endS - a.s;

        const double t = segmentLength > 1e-12
            ? std::clamp(
                (referenceS - a.s) / segmentLength,
                0.0,
                1.0
            )
            : 0.0;

        // Linear interpolation of track position.
        const double chassisRefX =
            a.x + t * (b.x - a.x);

        const double chassisRefY =
            a.y + t * (b.y - a.y);

        // Interpolate heading via the shortest angle.
        const double headingDifference = std::atan2(
            std::sin(b.heading - a.heading),
            std::cos(b.heading - a.heading)
        );

        const double heading =
            a.heading + t * headingDifference;

        // Convert chassis reference to rear axle.
        const double rearX =
            chassisRefX -
            rearAxleOffset_ * std::cos(heading);

        const double rearY =
            chassisRefY -
            rearAxleOffset_ * std::sin(heading);

        references.push_back({
            rearX,
            rearY,
            std::atan2(std::sin(heading), std::cos(heading)),
            targetSpeed
        });
    }

    return references;
}
