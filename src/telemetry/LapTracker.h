#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

class LapTracker {
public:
    explicit LapTracker(const std::string& waypointFile) {
        std::ifstream file(waypointFile);
        if (!file) {
            throw std::runtime_error("Cannot open track: " + waypointFile);
        }

        std::string line;
        std::getline(file, line);  // CSV header

        while (std::getline(file, line)) {
            const auto comma = line.find(',');
            if (comma == std::string::npos) continue;

            const auto secondComma = line.find(',', comma + 1);
            if (secondComma == std::string::npos) continue;

            Point point;
            point.x = std::stod(line.substr(0, comma));
            point.y = std::stod(
                line.substr(comma + 1, secondComma - comma - 1)
            );
            points_.push_back(point);
        }

        if (points_.size() < 3) {
            throw std::runtime_error("Insufficient track waypoints");
        }

        cumulative_.push_back(0.0);

        for (std::size_t i = 1; i < points_.size(); ++i) {
            totalLength_ += distance(points_[i - 1], points_[i]);
            cumulative_.push_back(totalLength_);
        }

        totalLength_ += distance(points_.back(), points_.front());
    }

    void reset() {
        initialized_ = false;
        timingStarted_ = false;
        completedLaps_ = 0;
        previousProgress_ = 0.0;
        forwardDistance_ = 0.0;
        lapStartTime_ = 0.0;
        lastLapTime_ = 0.0;
        bestLapTime_ = 0.0;
    }

    void update(double x, double y, double simulationTime) {
        const double progress = projectProgress(x, y);

        if (!initialized_) {
            previousProgress_ = progress;
            lapStartTime_ = simulationTime;
            initialized_ = true;
            timingStarted_ = true;
            return;
        }

        double delta = progress - previousProgress_;

        // Unwrap progress around the closed circuit.
        if (delta < -totalLength_ / 2.0) {
            delta += totalLength_;
        } else if (delta > totalLength_ / 2.0) {
            delta -= totalLength_;
        }

        // Ignore implausible projection jumps.
        if (std::abs(delta) > 5.0) {
            previousProgress_ = progress;
            return;
        }

        // Signed progress prevents repeated forward/backward
        // movement from being mistaken for a completed lap.
        forwardDistance_ += delta;
        previousProgress_ = progress;

        if (forwardDistance_ >= totalLength_) {
            const double lapTime =
                simulationTime - lapStartTime_;

            if (lapTime > 0.0) {
                lastLapTime_ = lapTime;

                if (bestLapTime_ == 0.0 ||
                    lapTime < bestLapTime_) {
                    bestLapTime_ = lapTime;
                }

                ++completedLaps_;
                lapStartTime_ = simulationTime;
                forwardDistance_ -= totalLength_;
            }
        }
    }

    int completedLaps() const { return completedLaps_; }
    bool timingStarted() const { return timingStarted_; }

    double currentLapTime(double simulationTime) const {
        return timingStarted_
            ? std::max(0.0, simulationTime - lapStartTime_)
            : 0.0;
    }

    double lastLapTime() const { return lastLapTime_; }
    double bestLapTime() const { return bestLapTime_; }
    double trackLength() const { return totalLength_; }

private:
    struct Point {
        double x;
        double y;
    };

    static double distance(const Point& a, const Point& b) {
        return std::hypot(b.x - a.x, b.y - a.y);
    }

    double projectProgress(double x, double y) const {
        double bestDistance = std::numeric_limits<double>::infinity();
        double bestProgress = 0.0;

        for (std::size_t i = 0; i < points_.size(); ++i) {
            const std::size_t next = (i + 1) % points_.size();

            const auto& a = points_[i];
            const auto& b = points_[next];

            const double dx = b.x - a.x;
            const double dy = b.y - a.y;
            const double lengthSquared = dx * dx + dy * dy;

            if (lengthSquared < 1e-12) continue;

            const double t = std::clamp(
                ((x - a.x) * dx + (y - a.y) * dy) / lengthSquared,
                0.0, 1.0
            );

            const double px = a.x + t * dx;
            const double py = a.y + t * dy;

            const double d2 =
                (x - px) * (x - px) +
                (y - py) * (y - py);

            if (d2 < bestDistance) {
                bestDistance = d2;
                bestProgress =
                    cumulative_[i] + t * std::sqrt(lengthSquared);
            }
        }

        return bestProgress >= totalLength_
            ? 0.0
            : bestProgress;
    }

    std::vector<Point> points_;
    std::vector<double> cumulative_;

    double totalLength_ = 0.0;
    double previousProgress_ = 0.0;
    double forwardDistance_ = 0.0;
    double lapStartTime_ = 0.0;
    double lastLapTime_ = 0.0;
    double bestLapTime_ = 0.0;

    int completedLaps_ = 0;
    bool initialized_ = false;
    bool timingStarted_ = false;
};
