#pragma once

#include <algorithm>
#include <stdexcept>

class LongitudinalControlAdapter {
public:
    explicit LongitudinalControlAdapter(
        double torquePerAcceleration = 140.0,
        double maxTorque = 500.0
    )
        : gain_(torquePerAcceleration),
          maxTorque_(maxTorque) {
        if (gain_ <= 0.0 || maxTorque_ <= 0.0) {
            throw std::invalid_argument(
                "Longitudinal adapter parameters must be positive"
            );
        }
    }

    // Returns commanded torque for EACH rear wheel.
    double toTorque(double desiredAcceleration) const {
        return std::clamp(
            gain_ * desiredAcceleration,
            -maxTorque_,
            maxTorque_
        );
    }

private:
    double gain_;
    double maxTorque_;
};
