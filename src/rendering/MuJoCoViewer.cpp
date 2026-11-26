#include "telemetry/LapTracker.h"
#include <cstdio>

#include "MuJoCoViewer.h"

#include <algorithm>
#include <iostream>
#include <stdexcept>
#include "controllers/PurePursuit.h"

// ============================================
// Constructor
// ============================================

MuJoCoViewer::MuJoCoViewer(
    MuJoCoSimulator& simulator
) : simulator_(simulator) {

    if (!glfwInit()) {
        throw std::runtime_error(
            "GLFW initialization failed"
        );
    }

    window_ = glfwCreateWindow(
        1280,
        800,
        "F1 MuJoCo - Autonomous Racing",
        nullptr,
        nullptr
    );

    if (!window_) {
        glfwTerminate();
        throw std::runtime_error(
            "Failed to create GLFW window"
        );
    }

    // MuJoCo rendering requires an active
    // OpenGL context.
    glfwMakeContextCurrent(window_);
    glfwSwapInterval(1);

    mjv_defaultCamera(&camera_);
    mjv_defaultOption(&option_);

    mjv_makeScene(
        simulator_.model(),
        &scene_,
        3000
    );

    mjr_defaultContext(&context_);

    mjr_makeContext(
        simulator_.model(),
        &context_,
        mjFONTSCALE_150
    );

    // Initial camera: follow the chassis.
    int chassisId = mj_name2id(
        simulator_.model(),
        mjOBJ_BODY,
        "chassis"
    );

    if (chassisId < 0) {
        throw std::runtime_error(
            "Chassis body not found"
        );
    }

    camera_.type = mjCAMERA_TRACKING;
    camera_.trackbodyid = chassisId;
    camera_.distance = 23.0;
    camera_.azimuth = 135.0;
    camera_.elevation = -25.0;

    std::cout << "\n3D viewer initialized.\n";
}

// ============================================
// Destructor
// ============================================

MuJoCoViewer::~MuJoCoViewer() {

    if (window_) {
        glfwMakeContextCurrent(window_);
    }

    mjr_freeContext(&context_);
    mjv_freeScene(&scene_);

    if (window_) {
        glfwDestroyWindow(window_);
    }

    glfwTerminate();
}

// ============================================
// Real-time simulation + rendering
// ============================================

void MuJoCoViewer::run() {

    std::cout << "\nControls:\n";
    std::cout << "  LEFT / RIGHT : Steering\n";
    std::cout << "  SPACE        : Pause / Resume\n";
    std::cout << "  R            : Reset\n";
    std::cout << "  C            : Change camera\n";
    std::cout << "  ESC          : Exit\n\n";

    constexpr double targetSpeed = 4.0;
    constexpr double Kp = 100.0;
    constexpr double maxTorque = 40.0;
    constexpr double steeringCommand = 0.1;

    double previousTime = glfwGetTime();
    double accumulator = 0.0;

    bool wasSpaceDown = false;
    bool wasResetDown = false;
    bool wasCameraDown = false;

    PurePursuit purePursuit(
        "tracks/custom_waypoints.csv",
        3.6,
        4.0
    );

    LapTracker lapTracker("tracks/custom_waypoints.csv");

    bool autonomousMode = false;
    bool wasADown = false;
    bool wasMDown = false;

    std::cout << "Loaded "
            << purePursuit.waypointCount()
            << " racing waypoints.\n";

    while (!glfwWindowShouldClose(window_)) {

        glfwPollEvents();

        bool aDown =
            glfwGetKey(window_, GLFW_KEY_A) == GLFW_PRESS;

        bool mDown =
            glfwGetKey(window_, GLFW_KEY_M) == GLFW_PRESS;

        if (aDown && !wasADown) {
            autonomousMode = true;
            std::cout << "AUTO: Pure Pursuit enabled\n";
        }

        if (mDown && !wasMDown) {
            autonomousMode = false;
            std::cout << "MANUAL: Keyboard steering enabled\n";
        }

        wasADown = aDown;
        wasMDown = mDown;

        static bool previousLeft = false;
        static bool previousRight = false;

        bool leftPressed =
            glfwGetKey(window_, GLFW_KEY_LEFT) == GLFW_PRESS;

        bool rightPressed =
            glfwGetKey(window_, GLFW_KEY_RIGHT) == GLFW_PRESS;

        if (leftPressed != previousLeft) {
            std::cout << "LEFT arrow: "
                    << (leftPressed ? "PRESSED" : "RELEASED")
                    << std::endl;
        }

        if (rightPressed != previousRight) {
            std::cout << "RIGHT arrow: "
                    << (rightPressed ? "PRESSED" : "RELEASED")
                    << std::endl;
        }

        previousLeft = leftPressed;
        previousRight = rightPressed;


        if (glfwGetKey(window_, GLFW_KEY_ESCAPE)
            == GLFW_PRESS) {
            glfwSetWindowShouldClose(window_, GLFW_TRUE);
        }

        // Current keyboard states.
        bool spaceDown =
            glfwGetKey(window_, GLFW_KEY_SPACE)
            == GLFW_PRESS;

        bool resetDown =
            glfwGetKey(window_, GLFW_KEY_R)
            == GLFW_PRESS;

        bool cameraDown =
            glfwGetKey(window_, GLFW_KEY_C)
            == GLFW_PRESS;

        // Toggle pause on a new key press.
        if (spaceDown && !wasSpaceDown) {
            paused_ = !paused_;
        }

        // Reset the physical simulation.
        if (resetDown && !wasResetDown) {
            simulator_.reset();
            lapTracker.reset();
            accumulator = 0.0;
            std::cout << "Lap timing reset.\n";
        }

        // Toggle between chase and aerial views.
        if (cameraDown && !wasCameraDown) {
            overhead_ = !overhead_;

            if (overhead_) {
                camera_.type = mjCAMERA_FREE;
                camera_.lookat[0] = 0.0;
                camera_.lookat[1] = 0.0;
                camera_.lookat[2] = 0.0;
                camera_.distance = 290.0;
                camera_.azimuth = 90.0;
                camera_.elevation = -89.0;
            } else {
                camera_.type = mjCAMERA_TRACKING;
                camera_.trackbodyid = mj_name2id(
                    simulator_.model(),
                    mjOBJ_BODY,
                    "chassis"
                );

                camera_.distance = 23.0;
                camera_.azimuth = 135.0;
                camera_.elevation = -25.0;
            }
        }

        wasSpaceDown = spaceDown;
        wasResetDown = resetDown;
        wasCameraDown = cameraDown;

        // Measure real elapsed time.
        double now = glfwGetTime();
        double elapsed = now - previousTime;
        previousTime = now;

        // Limit catch-up after dragging or pausing.
        elapsed = std::clamp(elapsed, 0.0, 0.05);

        if (!paused_) {
            accumulator += elapsed;

            const double timestep =
                simulator_.model()->opt.timestep;

            // Fixed physics timestep.

            while (accumulator >= timestep) {

                // ==========================================
                // 1. Read current vehicle state
                // ==========================================
                VehicleState state = simulator_.getVehicleState();

                // ==========================================
                // 2. Longitudinal speed controller
                // ==========================================
                // Predict a safe target speed from upcoming corners.
                // Manual mode retains the original fixed speed.

                double desiredSpeed = targetSpeed;

                if (autonomousMode) {

                    // Existing predictive curvature-based speed planning.
                    const double curvatureSpeed =
                        purePursuit.computeTargetSpeed(state, targetSpeed);

                    // Pure Pursuit steering demand.
                    const double steeringDemand =
                        std::abs(purePursuit.computeSteering(state));

                    // Gradually reduce speed when steering demand is high.
                    constexpr double steeringStart = 0.15;
                    constexpr double steeringFull = 0.30;
                    constexpr double minimumCornerSpeed = 1.0;

                    const double severity = std::clamp(
                        (steeringDemand - steeringStart) /
                        (steeringFull - steeringStart),
                        0.0,
                        1.0
                    );

                    const double steeringSpeed =
                        targetSpeed -
                        severity * (targetSpeed - minimumCornerSpeed);

                    desiredSpeed = std::min(
                        curvatureSpeed,
                        steeringSpeed
                    );
                }

                const double speedError = desiredSpeed - state.speed;

                // Permit negative rear-wheel torque for braking.
                constexpr double maxBrakingTorque = 30.0;

                const double torque = std::clamp(
                    Kp * speedError,
                    -maxBrakingTorque,
                    maxTorque
                );

                simulator_.setRearTorque(torque);

                // ==========================================
                // 3. Steering controller
                // ==========================================
                double rawSteering = 0.0;

                constexpr double manualSteering = 0.1;
                constexpr double autonomousSteeringLimit = 0.3;

                if (autonomousMode) {

                    // Pure Pursuit computes the requested
                    // steering angle from the racing waypoints.
                    rawSteering = purePursuit.computeSteering(state);

                } else {

                    // Manual steering using keyboard arrows.
                    const bool leftPressed =
                        glfwGetKey(window_, GLFW_KEY_LEFT) == GLFW_PRESS;

                    const bool rightPressed =
                        glfwGetKey(window_, GLFW_KEY_RIGHT) == GLFW_PRESS;

                    if (leftPressed && !rightPressed) {
                        rawSteering = manualSteering;
                    } else if (rightPressed && !leftPressed) {
                        rawSteering = -manualSteering;
                    }
                }

                // Limit automatic steering independently
                // from the smaller manual steering command.
                const double steeringLimit =
                    autonomousMode ? autonomousSteeringLimit
                                : manualSteering;

                const double appliedSteering = std::clamp(
                    rawSteering,
                    -steeringLimit,
                    steeringLimit
                );

                // Apply steering only once per physics step.
                simulator_.setSteering(appliedSteering);

                // ==========================================
                // 4. Advance MuJoCo physics
                // ==========================================
                simulator_.step();

                // ==========================================
                // 5. Read updated state and diagnostics
                // ==========================================
                static double nextDiagnosticTime = 0.0;

                if (simulator_.getSimulationTime() >= nextDiagnosticTime) {

                    const VehicleState current =
                        simulator_.getVehicleState();

                    const double cte =
                        purePursuit.crossTrackError(current);

                                        
                    // Find front steering joints.
                    const mjModel* model = simulator_.model();
                    mjData* data = simulator_.data();

                    const int steerFL = mj_name2id(
                        model, mjOBJ_JOINT, "steer_FL"
                    );

                    const int steerFR = mj_name2id(
                        model, mjOBJ_JOINT, "steer_FR"
                    );

                    if (steerFL < 0 || steerFR < 0) {
                        throw std::runtime_error(
                            "Front steering joints not found"
                        );
                    }

                    // Read actual steering-joint positions.
                    const double actualFL =
                        data->qpos[model->jnt_qposadr[steerFL]];

                    const double actualFR =
                        data->qpos[model->jnt_qposadr[steerFR]];

                    
                    // =============================================
                    // Read steering actuator forces
                    // =============================================

                    const int actuatorFL = mj_name2id(
                        model,
                        mjOBJ_ACTUATOR,
                        "steering_FL"
                    );

                    const int actuatorFR = mj_name2id(
                        model,
                        mjOBJ_ACTUATOR,
                        "steering_FR"
                    );

                    if (actuatorFL < 0 || actuatorFR < 0) {
                        throw std::runtime_error(
                            "Steering actuator not found"
                        );
                    }

                    const double forceFL =
                        data->actuator_force[actuatorFL];

                    const double forceFR =
                        data->actuator_force[actuatorFR];

                    // Target minus actual steering angle.
                    const double steeringErrorFL =
                        appliedSteering - actualFL;

                    const double steeringErrorFR =
                        appliedSteering - actualFR;


                    
                    if (cte > 5.5) {
                        std::cout
                            << "WARNING: Vehicle approaching track boundary!"
                            << std::endl;
                    }

                    const bool steeringSaturated =
                        autonomousMode &&
                        std::abs(rawSteering) >
                            autonomousSteeringLimit;


                    std::cout
                        << "Mode: "
                        << (autonomousMode ? "AUTO" : "MANUAL")
                        << " | Time: " << simulator_.getSimulationTime()
                        << " | X: " << current.x
                        << " | Y: " << current.y
                        << " | Heading: " << current.heading
                        << " | Speed: " << current.speed
                        << " | Target Speed: " << desiredSpeed
                        << " | Raw Steering: " << rawSteering
                        << " | Applied Steering: " << appliedSteering
                        << " | Actual FL: " << actualFL
                        << " | Actual FR: " << actualFR
                        << " | Force FL: " << forceFL
                        << " | Force FR: " << forceFR
                        << " | Error FL: " << steeringErrorFL
                        << " | Error FR: " << steeringErrorFR
                        << " | Saturated: "
                        << (steeringSaturated ? "YES" : "NO")
                        << " | CTE: " << cte << " m"
                        << " | Torque: " << torque
                        << std::endl;

                    nextDiagnosticTime =
                        simulator_.getSimulationTime() + 1.0;
                }

                // ==========================================
                // 6. Update lap tracking
                // ==========================================
                {
                    const VehicleState lapState =
                        simulator_.getVehicleState();

                    lapTracker.update(
                        lapState.x,
                        lapState.y,
                        simulator_.getSimulationTime()
                    );
                }

                // ==========================================
                // 7. Consume one simulation timestep
                // ==========================================
                accumulator -= timestep;
            }

        } else {
            accumulator = 0.0;
        }

        // Construct visual scene from current state.
        mjv_updateScene(
            simulator_.model(),
            simulator_.data(),
            &option_,
            nullptr,
            &camera_,
            mjCAT_ALL,
            &scene_
        );

        mjrRect viewport = {0, 0, 0, 0};

        glfwGetFramebufferSize(
            window_,
            &viewport.width,
            &viewport.height
        );

        mjr_render(
            viewport,
            &scene_,
            &context_
        );

        // ==========================================
        // Live telemetry HUD
        // ==========================================
        {
            const VehicleState hudState =
                simulator_.getVehicleState();

            const double speedKmh = hudState.speed * 3.6;

            const double hudCTE =
                purePursuit.crossTrackError(hudState);

            const double simTime =
                simulator_.getSimulationTime();

            char currentLap[32] = "WAITING";
            char lastLap[32] = "--";
            char bestLap[32] = "--";

            if (lapTracker.timingStarted()) {
                std::snprintf(
                    currentLap,
                    sizeof(currentLap),
                    "%.2f s",
                    lapTracker.currentLapTime(simTime)
                );
            }

            if (lapTracker.lastLapTime() > 0.0) {
                std::snprintf(
                    lastLap,
                    sizeof(lastLap),
                    "%.2f s",
                    lapTracker.lastLapTime()
                );
            }

            if (lapTracker.bestLapTime() > 0.0) {
                std::snprintf(
                    bestLap,
                    sizeof(bestLap),
                    "%.2f s",
                    lapTracker.bestLapTime()
                );
            }

            char hud[512];

            std::snprintf(
                hud,
                sizeof(hud),
                "MODE: %s\n"
                "SPEED: %.2f km/h\n"
                "CTE: %.3f m\n"
                "SIM TIME: %.1f s\n"
                "\n"
                "LAPS COMPLETED: %d\n"
                "CURRENT LAP: %s\n"
                "LAST LAP: %s\n"
                "BEST LAP: %s",
                autonomousMode ? "AUTO" : "MANUAL",
                speedKmh,
                hudCTE,
                simTime,
                lapTracker.completedLaps(),
                currentLap,
                lastLap,
                bestLap
            );

            mjr_overlay(
                mjFONT_NORMAL,
                mjGRID_TOPLEFT,
                viewport,
                hud,
                nullptr,
                &context_
            );
        }

        glfwSwapBuffers(window_);
    }
}
