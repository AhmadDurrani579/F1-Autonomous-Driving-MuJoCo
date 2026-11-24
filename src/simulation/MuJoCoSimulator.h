
#pragma once

#include <mujoco/mujoco.h>
#include <string>

struct VehicleState {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0; // radians
    double speed = 0.0;   // m/s
};

class MuJoCoSimulator {
public:
    explicit MuJoCoSimulator(const std::string& model_path);
    ~MuJoCoSimulator();

    MuJoCoSimulator(const MuJoCoSimulator&) = delete;
    MuJoCoSimulator& operator=(const MuJoCoSimulator&) = delete;

    VehicleState getVehicleState() const;

    void setSteering(double angle);
    void setRearTorque(double torque);
    const mjModel* model() const { return model_; }
    mjData* data() const { return data_; }

    void step();
    void reset();

    double getSimulationTime() const;

private:
    mjModel* model_ = nullptr;
    mjData* data_ = nullptr;

    int chassisId_ = -1;
    int chassisDofAdr_ = -1;

    int steeringFL_ = -1;
    int steeringFR_ = -1;

    int motorRL_ = -1;
    int motorRR_ = -1;
};