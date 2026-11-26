#pragma once

struct MPCConfig {
    // Prediction settings
    int horizon = 20;
    double dt = 0.1;

    // Experimental actuator-aware MPC.
    // Disabled by default.
    bool enableSteeringDynamics = true;
    double steeringResponseTime = 0.8;

    // Vehicle geometry
    double wheelbase = 3.6;

    // Control constraints
    double maxSteering = 0.3;
    double maxSteeringRate = 0.5; // radians per second
    double minAcceleration = -2.0;
    double maxAcceleration = 2.0;

    // Initial speed limit (m/s)
    double maxSpeed = 3.0;

    // Cost-function weights (initial values)
    double positionWeight = 10.0;
    double headingWeight = 5.0;
    double speedWeight = 2.0;
    double steeringWeight = 0.1;
    double steeringSmoothnessWeight = 10.0;
    double accelerationWeight = 0.1;
};
