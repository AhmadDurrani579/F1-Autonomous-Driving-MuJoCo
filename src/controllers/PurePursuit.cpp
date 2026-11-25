
#include "PurePursuit.h"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <limits>
#include <sstream>
#include <stdexcept>

// ==================================================
// Load racing circuit waypoints
// ==================================================

PurePursuit::PurePursuit(
    const std::string& waypointFile,
    double wheelbase,
    double lookahead
)
    : wheelbase_(wheelbase),
      lookahead_(lookahead) {

    if (wheelbase_ <= 0.0 || lookahead_ <= 0.0) {
        throw std::invalid_argument(
            "Wheelbase and lookahead must be positive"
        );
    }

    std::ifstream file(waypointFile);

    if (!file.is_open()) {
        throw std::runtime_error(
            "Cannot open waypoint file: " + waypointFile
        );
    }

    std::string line;

    // Skip CSV header: x,y,heading,s
    std::getline(file, line);

    while (std::getline(file, line)) {
        if (line.empty()) {
            continue;
        }

        std::stringstream ss(line);
        std::string xText;
        std::string yText;

        if (!std::getline(ss, xText, ',') ||
            !std::getline(ss, yText, ',')) {
            throw std::runtime_error(
                "Malformed waypoint row: " + line
            );
        }

        Waypoint point;
        point.x = std::stod(xText);
        point.y = std::stod(yText);

        waypoints_.push_back(point);
    }

    if (waypoints_.size() < 3) {
        throw std::runtime_error(
            "At least three waypoints are required"
        );
    }
}

// ==================================================
// Calculate steering angle
// ==================================================



double PurePursuit::computeSteering(
    const VehicleState& state
) {
    const std::size_t count = waypoints_.size();

    if (count < 2) {
        return 0.0;
    }

    // Find the closest projection of the vehicle
    // onto any segment of the closed racing line.
    std::size_t nearestSegment = 0;
    double nearestT = 0.0;
    double bestDistanceSquared =
        std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < count; ++i) {
        const std::size_t next = (i + 1) % count;

        const Waypoint& a = waypoints_[i];
        const Waypoint& b = waypoints_[next];

        const double sx = b.x - a.x;
        const double sy = b.y - a.y;
        const double lengthSquared = sx * sx + sy * sy;

        if (lengthSquared < 1e-12) {
            continue;
        }

        const double px = state.x - a.x;
        const double py = state.y - a.y;

        const double t = std::clamp(
            (px * sx + py * sy) / lengthSquared,
            0.0,
            1.0
        );

        const double projectedX = a.x + t * sx;
        const double projectedY = a.y + t * sy;

        const double dx = state.x - projectedX;
        const double dy = state.y - projectedY;

        const double distanceSquared = dx * dx + dy * dy;

        if (distanceSquared < bestDistanceSquared) {
            bestDistanceSquared = distanceSquared;
            nearestSegment = i;
            nearestT = t;
        }
    }

    // Travel lookahead_ metres forward from
    // the projected position along the racing line.
    std::size_t index = nearestSegment;
    double remainingDistance = lookahead_;
    double startT = nearestT;

    Waypoint targetPoint = waypoints_[index];

    for (std::size_t step = 0; step < count; ++step) {
        const std::size_t next = (index + 1) % count;

        const Waypoint& a = waypoints_[index];
        const Waypoint& b = waypoints_[next];

        const double sx = b.x - a.x;
        const double sy = b.y - a.y;
        const double segmentLength = std::hypot(sx, sy);

        if (segmentLength < 1e-9) {
            index = next;
            startT = 0.0;
            continue;
        }

        const double availableDistance =
            (1.0 - startT) * segmentLength;

        if (remainingDistance <= availableDistance) {
            const double t =
                startT + remainingDistance / segmentLength;

            targetPoint.x = a.x + t * sx;
            targetPoint.y = a.y + t * sy;

            targetIndex_ = next;
            break;
        }

        remainingDistance -= availableDistance;
        index = next;
        startT = 0.0;
        targetIndex_ = index;
    }

    // Calculate the direction to the target.
    const double dx = targetPoint.x - state.x;
    const double dy = targetPoint.y - state.y;

    const double alpha =
        std::atan2(dy, dx) - state.heading;

    const double targetDistance =
        std::max(0.1, std::hypot(dx, dy));

    // Pure Pursuit bicycle-model steering.
    return std::atan2(
        2.0 * wheelbase_ * std::sin(alpha),
        targetDistance
    );
}



double PurePursuit::crossTrackError(
    const VehicleState& state
) const {

    double minimumDistance =
        std::numeric_limits<double>::infinity();

    const std::size_t count = waypoints_.size();

    for (std::size_t i = 0; i < count; ++i) {

        // Current path segment
        const Waypoint& a = waypoints_[i];
        const Waypoint& b = waypoints_[(i + 1) % count];

        const double dx = b.x - a.x;
        const double dy = b.y - a.y;

        const double lengthSquared =
            dx * dx + dy * dy;

        if (lengthSquared < 1e-12) {
            continue;
        }

        // Project vehicle position onto segment
        double t = (
            (state.x - a.x) * dx +
            (state.y - a.y) * dy
        ) / lengthSquared;

        // Keep projection within the segment
        t = std::clamp(t, 0.0, 1.0);

        // Nearest point on this segment
        const double closestX = a.x + t * dx;
        const double closestY = a.y + t * dy;

        // Distance from vehicle to segment
        const double distance = std::hypot(
            state.x - closestX,
            state.y - closestY
        );

        minimumDistance = std::min(
            minimumDistance,
            distance
        );
    }

    return minimumDistance;
}


double PurePursuit::computeTargetSpeed(
    const VehicleState& state,
    double maximumSpeed
) const {
    constexpr double minimumSpeed = 0.9;
    constexpr double maxLateralAccel = 0.20;
    constexpr double brakingAccel = 0.5;
    constexpr double planningDistance = 30.0;
    constexpr double sampleStep = 2.0;
    constexpr double curvatureSpacing = 3.0;
    constexpr double brakingBuffer = 5.0;

    const std::size_t count = waypoints_.size();

    // Find the waypoint closest to the car.
    std::size_t nearest = 0;
    double bestDistance = std::numeric_limits<double>::infinity();

    for (std::size_t i = 0; i < count; ++i) {
        const double dx = waypoints_[i].x - state.x;
        const double dy = waypoints_[i].y - state.y;
        const double d2 = dx * dx + dy * dy;

        if (d2 < bestDistance) {
            bestDistance = d2;
            nearest = i;
        }
    }

    // Interpolate a position a specified distance
    // forward along the closed waypoint path.
    const auto pointAhead = [&](double distance) -> Waypoint {
        std::size_t index = nearest;

        for (std::size_t step = 0; step < count; ++step) {
            const std::size_t next = (index + 1) % count;

            const Waypoint& a = waypoints_[index];
            const Waypoint& b = waypoints_[next];

            const double length = std::hypot(
                b.x - a.x,
                b.y - a.y
            );

            if (length < 1e-9) {
                index = next;
                continue;
            }

            if (distance <= length) {
                const double t = distance / length;

                return {
                    a.x + t * (b.x - a.x),
                    a.y + t * (b.y - a.y)
                };
            }

            distance -= length;
            index = next;
        }

        return waypoints_[index];
    };

    double plannedSpeed = maximumSpeed;

    // Check upcoming corners and calculate the
    // maximum speed allowed at the current position.
    for (double d = 0.0;
         d <= planningDistance;
         d += sampleStep) {

        const Waypoint p0 = pointAhead(d);
        const Waypoint p1 = pointAhead(d + curvatureSpacing);
        const Waypoint p2 = pointAhead(d + 2.0 * curvatureSpacing);

        const double a = std::hypot(
            p1.x - p0.x,
            p1.y - p0.y
        );

        const double b = std::hypot(
            p2.x - p1.x,
            p2.y - p1.y
        );

        const double c = std::hypot(
            p2.x - p0.x,
            p2.y - p0.y
        );

        if (a * b * c < 1e-9) {
            continue;
        }

        const double cross =
            (p1.x - p0.x) * (p2.y - p0.y) -
            (p1.y - p0.y) * (p2.x - p0.x);

        const double curvature =
            std::abs(2.0 * cross / (a * b * c));

        if (curvature < 1e-6) {
            continue;
        }

        // Maximum speed permitted by curvature.
        const double cornerSpeed = std::clamp(
            std::sqrt(maxLateralAccel / curvature),
            minimumSpeed,
            maximumSpeed
        );

        // Allow for braking before reaching the corner.
        const double availableDistance =
            std::max(0.0, d - brakingBuffer);

        const double allowedSpeed = std::sqrt(
            cornerSpeed * cornerSpeed +
            2.0 * brakingAccel * availableDistance
        );

        plannedSpeed = std::min(plannedSpeed, allowedSpeed);
    }

    return std::clamp(
        plannedSpeed,
        minimumSpeed,
        maximumSpeed
    );
}
