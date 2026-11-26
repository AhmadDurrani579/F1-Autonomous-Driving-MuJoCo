#pragma once

#include "MPCConfig.h"
#include "MPCCost.h"
#include "../models/KinematicBicycleModel.h"

#include <cstddef>
#include <string>
#include <vector>

class MPCTrackReference {
public:
    explicit MPCTrackReference(
        const std::string& waypointFile,
        double rearAxleOffset = 1.8
    );

    std::vector<MPCReference> generate(
        const BicycleState& currentState,
        const MPCConfig& config,
        double targetSpeed = 2.0
    ) const;

    std::size_t waypointCount() const {
        return waypoints_.size();
    }

private:
    struct TrackWaypoint {
        double x = 0.0;
        double y = 0.0;
        double heading = 0.0;
        double s = 0.0;
    };

    std::vector<TrackWaypoint> waypoints_;

    double rearAxleOffset_;
    double trackLength_ = 0.0;
};
