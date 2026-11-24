
#include "MuJoCoSimulator.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <string>

// =====================================================
// Constructor: Load the Formula 1 simulation
// =====================================================

MuJoCoSimulator::MuJoCoSimulator(
    const std::string& model_path
) {
    char error[1024] = {};

    model_ = mj_loadXML(
        model_path.c_str(),
        nullptr,
        error,
        sizeof(error)
    );

    if (!model_) {
        throw std::runtime_error(
            "Failed to load MuJoCo model: " +
            std::string(error)
        );
    }

    // Find the chassis body by name.
    chassisId_ = mj_name2id(
        model_,
        mjOBJ_BODY,
        "chassis"
    );

    if (chassisId_ < 0) {
        mj_deleteModel(model_);
        model_ = nullptr;
        throw std::runtime_error(
            "Body 'chassis' was not found in the XML"
        );
    }

    // Locate the chassis free joint.
    int jointAdr = model_->body_jntadr[chassisId_];
    int jointCount = model_->body_jntnum[chassisId_];

    if (jointCount < 1 ||
        model_->jnt_type[jointAdr] != mjJNT_FREE) {

        mj_deleteModel(model_);
        model_ = nullptr;

        throw std::runtime_error(
            "Chassis must have a free joint"
        );
    }

    chassisDofAdr_ = model_->jnt_dofadr[jointAdr];

    // Allocate the dynamic simulation state.
    data_ = mj_makeData(model_);

    if (!data_) {
        mj_deleteModel(model_);
        model_ = nullptr;

        throw std::runtime_error(
            "Failed to allocate MuJoCo data"
        );
    }

    // Calculate initial body positions and orientations.
    mj_forward(model_, data_);


    // Locate the four Formula 1 actuators.
    steeringFL_ = mj_name2id(
        model_, mjOBJ_ACTUATOR, "steering_FL"
    );

    steeringFR_ = mj_name2id(
        model_, mjOBJ_ACTUATOR, "steering_FR"
    );

    motorRL_ = mj_name2id(
        model_, mjOBJ_ACTUATOR, "rear_drive_RL"
    );

    motorRR_ = mj_name2id(
        model_, mjOBJ_ACTUATOR, "rear_drive_RR"
    );

    // Stop initialization if any actuator is missing.
    if (steeringFL_ < 0 || steeringFR_ < 0 ||
        motorRL_ < 0 || motorRR_ < 0) {

        mj_deleteData(data_);
        data_ = nullptr;

        mj_deleteModel(model_);
        model_ = nullptr;

        throw std::runtime_error(
            "One or more Formula 1 actuators were not found"
        );
    }


    std::cout << "MuJoCo version: "
              << mj_versionString() << '\n';

    std::cout << "Model loaded: "
              << model_path << '\n';

    std::cout << "Bodies: " << model_->nbody << '\n';
    std::cout << "Geometries: " << model_->ngeom << '\n';
    std::cout << "Meshes: " << model_->nmesh << '\n';

    // Inspect available actuators before assigning them.
    std::cout << "\nAvailable actuators:\n";

    for (int i = 0; i < model_->nu; ++i) {
        const char* name = mj_id2name(
            model_,
            mjOBJ_ACTUATOR,
            i
        );

        std::cout << "  ID " << i << ": "
                  << (name ? name : "(unnamed)")
                  << '\n';
    }

    std::cout << "\nSimulator initialized.\n";
}

// =====================================================
// Destructor: Release MuJoCo memory
// =====================================================

MuJoCoSimulator::~MuJoCoSimulator() {
    if (data_) {
        mj_deleteData(data_);
    }

    if (model_) {
        mj_deleteModel(model_);
    }
}

// =====================================================
// Read vehicle position, orientation and speed
// =====================================================

VehicleState MuJoCoSimulator::getVehicleState() const {
    VehicleState state;

    // Chassis position in world coordinates.
    const mjtNum* pos =
        data_->xpos + 3 * chassisId_;

    state.x = pos[0];
    state.y = pos[1];

    // Chassis rotation matrix in world coordinates.
    const mjtNum* rotation =
        data_->xmat + 9 * chassisId_;

    // Vehicle forward direction is local +X.
    state.heading = std::atan2(
        rotation[3],
        rotation[0]
    );

    // Free-joint translational velocity in world frame.
    const mjtNum* velocity =
        data_->qvel + chassisDofAdr_;

    state.speed = std::hypot(
        velocity[0],
        velocity[1]
    );

    return state;
}

// =====================================================
// Steering control
// =====================================================

void MuJoCoSimulator::setSteering(double angle) {
    if (steeringFL_ < 0 || steeringFR_ < 0) {
        throw std::runtime_error(
            "Steering actuator IDs are not configured yet"
        );
    }

    data_->ctrl[steeringFL_] = angle;
    data_->ctrl[steeringFR_] = angle;
}

// =====================================================
// Rear-wheel torque control
// =====================================================

void MuJoCoSimulator::setRearTorque(double torque) {
    if (motorRL_ < 0 || motorRR_ < 0) {
        throw std::runtime_error(
            "Rear motor actuator IDs are not configured yet"
        );
    }

    data_->ctrl[motorRL_] = torque;
    data_->ctrl[motorRR_] = torque;
}

// =====================================================
// Advance physics by one timestep
// =====================================================

void MuJoCoSimulator::step() {
    mj_step(model_, data_);
}

// =====================================================
// Reset the simulation
// =====================================================

void MuJoCoSimulator::reset() {
    mj_resetData(model_, data_);
    mj_forward(model_, data_);
}

// =====================================================
// Return current simulation time
// =====================================================

double MuJoCoSimulator::getSimulationTime() const {
    return data_->time;
}
