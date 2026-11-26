#include "studio/ux/plugin.h"
#include "imgui.h"
#include <atomic>
#include "controllers/PurePursuit.h"
#include "controllers/MPCController.h"
#include "models/LongitudinalControlAdapter.h"
#include "controllers/ControllerManager.h"
#include "telemetry/LapTracker.h"
#include <algorithm>
#include <cmath>
#include <memory>
#include <exception>

#include <mujoco/mujoco.h>
#include <cstdio>

namespace {

struct F1RacingPlugin {
    int chassisId = -1;
    bool modelReady = false;
    std::atomic<bool> autonomousMode{true};
    std::atomic<double> manualSteering{0.0};
    std::atomic<double> manualTorque{0.0};
    int chassisDofAdr = -1;
    int steeringFL = -1;
    int steeringFR = -1;
    int steerFLQposAdr = -1;
    int steerFRQposAdr = -1;
    int motorRL = -1;
    int motorRR = -1;
    ControllerManager controllerManager;
    std::unique_ptr<LapTracker> lapTracker;
    double currentSpeed = 0.0;
    double currentCTE = 0.0;
    double simulationTime = 0.0;
    double lastSteeringLogTime = -1.0;


    void OnModelLoaded(const mjModel* model) {
        chassisId = mj_name2id(
            model, mjOBJ_BODY, "chassis"
        );

        modelReady = false;
        lastSteeringLogTime = -1.0;
        controllerManager.clear();
        lapTracker.reset();

        chassisDofAdr = -1;
        steeringFL = -1;
        steeringFR = -1;
        steerFLQposAdr = -1;
        steerFRQposAdr = -1;

        if (chassisId < 0) return;

        const int jointAdr = model->body_jntadr[chassisId];

        if (model->body_jntnum[chassisId] < 1 ||
            model->jnt_type[jointAdr] != mjJNT_FREE) return;

        chassisDofAdr = model->jnt_dofadr[jointAdr];

        steeringFL = mj_name2id(
            model, mjOBJ_ACTUATOR, "steering_FL"
        );
        steeringFR = mj_name2id(
            model, mjOBJ_ACTUATOR, "steering_FR"
        );

        motorRL = mj_name2id(
            model, mjOBJ_ACTUATOR, "rear_drive_RL"
        );
        motorRR = mj_name2id(
            model, mjOBJ_ACTUATOR, "rear_drive_RR"
        );

        if (steeringFL < 0 || steeringFR < 0 ||
            motorRL < 0 || motorRR < 0) return;

        const int leftJointId = mj_name2id(
            model, mjOBJ_JOINT, "steer_FL"
        );
        const int rightJointId = mj_name2id(
            model, mjOBJ_JOINT, "steer_FR"
        );

        if (leftJointId < 0 || rightJointId < 0) return;

        steerFLQposAdr = model->jnt_qposadr[leftJointId];
        steerFRQposAdr = model->jnt_qposadr[rightJointId];

        controllerManager.addNamed(std::make_unique<PurePursuit>(
            "/Users/ahmad/projects/F1-MuJoCo/tracks/custom_waypoints.csv"
        ));

        // Register MPC as a second autonomous controller.
        // Keep Pure Pursuit as the default.
        try {
            controllerManager.addNamed(std::make_unique<MPCController>(
                "/Users/ahmad/projects/F1-MuJoCo/tracks/custom_waypoints.csv"
            ));
        } catch (const std::exception& error) {
            std::fprintf(
                stderr,
                "[F1 Racing] MPC registration failed: %s\n",
                error.what()
            );
        }


        lapTracker = std::make_unique<LapTracker>(
            "/Users/ahmad/projects/F1-MuJoCo/tracks/custom_waypoints.csv"
        );

        modelReady = true;

        if (modelReady) {
            std::printf(
                "[F1 Racing] Vehicle detected. "
                "Studio integration initialized.\n"
            );
        } else {
            std::printf(
                "[F1 Racing] No F1 chassis found. "
                "Controller disabled.\n"
            );
        }
    }

    void PreStep(const mjModel* model, mjData* data) {
        if (!modelReady) return;

        if (!autonomousMode.load()) {
            const double steering = manualSteering.load();
            const double torque = manualTorque.load();

            data->ctrl[steeringFL] = steering;
            data->ctrl[steeringFR] = steering;
            data->ctrl[motorRL] = torque;
            data->ctrl[motorRR] = torque;
            return;
        }

        DrivingController* controller = controllerManager.active();
        if (!controller) return;

        VehicleState state;

        const mjtNum* pos = data->xpos + 3 * chassisId;
        const mjtNum* rotation = data->xmat + 9 * chassisId;
        const mjtNum* velocity = data->qvel + chassisDofAdr;

        state.x = pos[0];
        state.y = pos[1];
        state.heading = std::atan2(rotation[3], rotation[0]);
        state.speed = std::hypot(velocity[0], velocity[1]);

        // ==========================================
        // MPC control: steering + optimised acceleration
        // ==========================================
        if (auto* mpc = dynamic_cast<MPCController*>(controller)) {

            // Read physical steering angles from MuJoCo.
            const double actualSteering =
                0.5 * (
                    data->qpos[steerFLQposAdr] +
                    data->qpos[steerFRQposAdr]
                );

            // MPC solves at 10 Hz, not every physics step.
            mpc->update(
                state,
                data->time,
                actualSteering
            );

            // Retain the latest command between updates.
            const BicycleControl control =
                mpc->latestControl();

            const bool valid = mpc->hasValidControl();

            const double steering = valid
                ? std::clamp(
                    control.steering,
                    -0.3,
                    0.3
                )
                : 0.0;

            static const LongitudinalControlAdapter
                torqueAdapter;

            // Convert desired acceleration to rear torque.
            // On failure, use neutral motor commands.
            const double rearTorque = valid
                ? torqueAdapter.toTorque(
                    control.acceleration
                )
                : 0.0;

            data->ctrl[steeringFL] = steering;
            data->ctrl[steeringFR] = steering;

            data->ctrl[motorRL] = rearTorque;
            data->ctrl[motorRR] = rearTorque;

            return;
        }

        // Existing Pure Pursuit logic remains unchanged.

        const double steering = controller->computeSteering(state);

        data->ctrl[steeringFL] = steering;
        data->ctrl[steeringFR] = steering;

        const double targetSpeed =
            controller->computeTargetSpeed(state, 3.0);

        const double speedError = targetSpeed - state.speed;
        const double rearTorque =
            std::clamp(150.0 * speedError, -100.0, 250.0);

        data->ctrl[motorRL] = rearTorque;
        data->ctrl[motorRR] = rearTorque;
    }

    void PostStep(const mjModel* model, mjData* data) {
        if (!modelReady) return;

        if (!lapTracker) return;

        const mjtNum* pos = data->xpos + 3 * chassisId;

        lapTracker->update(pos[0], pos[1], data->time);

        VehicleState state;
        const mjtNum* rotation = data->xmat + 9 * chassisId;
        const mjtNum* velocity = data->qvel + chassisDofAdr;

        state.x = pos[0];
        state.y = pos[1];
        state.heading = std::atan2(rotation[3], rotation[0]);
        state.speed = std::hypot(velocity[0], velocity[1]);

        currentSpeed = state.speed * 3.6;
        const DrivingController* controller = controllerManager.active();
        currentCTE = controller
            ? controller->crossTrackError(state)
            : 0.0;
        simulationTime = data->time;

        // Diagnostic: compare MPC steering targets
        // against actual MuJoCo steering-joint angles.
        if (autonomousMode.load() && controller) {

            if (data->time < lastSteeringLogTime) {
                lastSteeringLogTime = -1.0;
            }

            if (lastSteeringLogTime < 0.0 ||
                data->time - lastSteeringLogTime >= 0.099) {

                const int leftJoint = mj_name2id(
                    model, mjOBJ_JOINT, "steer_FL"
                );

                const int rightJoint = mj_name2id(
                    model, mjOBJ_JOINT, "steer_FR"
                );

                if (leftJoint >= 0 && rightJoint >= 0) {
                    const double leftCommand =
                        data->ctrl[steeringFL];

                    const double rightCommand =
                        data->ctrl[steeringFR];

                    const double leftActual =
                        data->qpos[
                            model->jnt_qposadr[leftJoint]
                        ];

                    const double rightActual =
                        data->qpos[
                            model->jnt_qposadr[rightJoint]
                        ];

                    std::printf(
                        "[F1 ACTUATOR] controller=%s t=%.2f "
                        "FL_cmd=%.4f FL_actual=%.4f FL_error=%.4f "
                        "FR_cmd=%.4f FR_actual=%.4f FR_error=%.4f\n",
                        controller->name().c_str(),
                        data->time,
                        leftCommand,
                        leftActual,
                        leftCommand - leftActual,
                        rightCommand,
                        rightActual,
                        rightCommand - rightActual
                    );
                }

                lastSteeringLogTime = data->time;
            }
        }
    }

    void UpdateGui() {
        bool autoEnabled = autonomousMode.load();

        if (ImGui::Checkbox("Autonomous Driving", &autoEnabled)) {
            manualSteering.store(0.0);
            manualTorque.store(0.0);
            autonomousMode.store(autoEnabled);
        }

        ImGui::Text("MODE: %s", autoEnabled ? "AUTO" : "MANUAL");

        ImGui::Separator();
        ImGui::TextUnformatted("Autonomous Controller");

        if (controllerManager.count() > 0) {
            const int current =
                static_cast<int>(controllerManager.selectedIndex());

            if (ImGui::BeginCombo(
                    "Controller",
                    controllerManager.name(current))) {

                for (std::size_t i = 0;
                     i < controllerManager.count(); ++i) {

                    const bool selected =
                        (i == controllerManager.selectedIndex());

                    if (ImGui::Selectable(
                            controllerManager.name(i), selected)) {
                        controllerManager.select(i);
                    }

                    if (selected) {
                        ImGui::SetItemDefaultFocus();
                    }
                }

                ImGui::EndCombo();
            }
        }


        if (!autoEnabled) {
            const bool focused =
                ImGui::IsWindowFocused(
                    ImGuiFocusedFlags_RootAndChildWindows
                ) && !ImGui::GetIO().WantTextInput;

            const bool left = focused && ImGui::IsKeyDown(ImGuiKey_A);
            const bool right = focused && ImGui::IsKeyDown(ImGuiKey_D);
            const bool forward = focused && ImGui::IsKeyDown(ImGuiKey_W);
            const bool backward = focused && ImGui::IsKeyDown(ImGuiKey_S);

            const double steering =
                (left ? 0.25 : 0.0) - (right ? 0.25 : 0.0);

            const double torque =
                (forward ? 200.0 : 0.0) -
                (backward ? 100.0 : 0.0);

            manualSteering.store(steering);
            manualTorque.store(torque);

            ImGui::TextUnformatted("Click this panel, then use W/A/S/D");
            ImGui::Text("Steering command: %.2f rad", steering);
            ImGui::Text("Motor command: %.1f", torque);
        } else {
            manualSteering.store(0.0);
            manualTorque.store(0.0);
        }
        ImGui::Separator();

        ImGui::Text("Speed: %.2f km/h", currentSpeed);
        ImGui::Text("Cross-Track Error: %.3f m", currentCTE);
        ImGui::Text("Simulation Time: %.1f s", simulationTime);

        ImGui::Separator();
        ImGui::Text("LAPS COMPLETED: %d",
                    lapTracker ? lapTracker->completedLaps() : 0);

        if (lapTracker && lapTracker->timingStarted()) {
            ImGui::Text("CURRENT LAP: %.2f s",
                lapTracker->currentLapTime(simulationTime));
        } else {
            ImGui::TextUnformatted("CURRENT LAP: --");
        }

        if (lapTracker && lapTracker->completedLaps() > 0) {
            ImGui::Text("LAST LAP: %.2f s",
                lapTracker->lastLapTime());
            ImGui::Text("BEST LAP: %.2f s",
                lapTracker->bestLapTime());
        } else {
            ImGui::TextUnformatted("LAST LAP: --");
            ImGui::TextUnformatted("BEST LAP: --");
        }
    }

};

}  // namespace

mjPLUGIN_LIB_INIT(f1_racing) {
    static F1RacingPlugin controller;

    mujoco::studio::ModelPlugin plugin;
    plugin.name = "F1 Racing";
    plugin.data = &controller;

    plugin.post_model_loaded =
        [](mujoco::studio::ModelPlugin* self,
           const mjModel* model,
           const char* model_path) {
            auto* controller =
                static_cast<F1RacingPlugin*>(self->data);
            controller->OnModelLoaded(model);
        };

    plugin.pre_step =
        [](mujoco::studio::ModelPlugin* self,
           const mjModel* model,
           mjData* data) {
            auto* controller =
                static_cast<F1RacingPlugin*>(self->data);
            controller->PreStep(model, data);
        };

    plugin.post_step =
        [](mujoco::studio::ModelPlugin* self,
           const mjModel* model,
           mjData* data) {
            auto* controller =
                static_cast<F1RacingPlugin*>(self->data);
            controller->PostStep(model, data);
        };

    mujoco::studio::RegisterPlugin(plugin);

    mujoco::studio::GuiPlugin gui;
    gui.name = "F1 Telemetry";
    gui.data = &controller;
    gui.active = true;
    gui.update = [](mujoco::studio::GuiPlugin* self) {
        auto* controller =
            static_cast<F1RacingPlugin*>(self->data);
        controller->UpdateGui();
    };

    mujoco::studio::RegisterPlugin(gui);

}
