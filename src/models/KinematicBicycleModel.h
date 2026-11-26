#pragma once

struct BicycleState {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;  // radians
    double speed = 0.0;    // metres per second
};

struct BicycleControl {
    double steering = 0.0;      // radians
    double acceleration = 0.0;  // metres per second squared
};

class KinematicBicycleModel {
public:
    explicit KinematicBicycleModel(double wheelbase);

    BicycleState predict(
        const BicycleState& state,
        const BicycleControl& control,
        double dt
    ) const;

private:
    double wheelbase_;
};
