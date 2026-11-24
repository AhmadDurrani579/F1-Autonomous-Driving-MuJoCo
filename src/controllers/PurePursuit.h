
#pragma once

#include "simulation/MuJoCoSimulator.h"

#include <cstddef>
#include <string>
#include <vector>

struct Waypoint {
    double x = 0.0;
    double y = 0.0;
};

class PurePursuit {
public:
    explicit PurePursuit(
        const std::string& waypointFile,
        double wheelbase = 3.6,
        double lookahead = 5.0
    );

    double computeSteering(const VehicleState& state);

    double crossTrackError(const VehicleState& state) const;

    double computeTargetSpeed(
        const VehicleState& state,
        double maximumSpeed = 2.0
    ) const;
    
    std::size_t targetIndex() const {
        return targetIndex_;
    }

    std::size_t waypointCount() const {
        return waypoints_.size();
    }

private:
    std::vector<Waypoint> waypoints_;

    double wheelbase_;
    double lookahead_;

    std::size_t targetIndex_ = 0;
};
