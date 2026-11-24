
#pragma once

#include "simulation/MuJoCoSimulator.h"

#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>

class MuJoCoViewer {
public:
    explicit MuJoCoViewer(MuJoCoSimulator& simulator);
    ~MuJoCoViewer();

    MuJoCoViewer(const MuJoCoViewer&) = delete;
    MuJoCoViewer& operator=(const MuJoCoViewer&) = delete;

    void run();

private:
    MuJoCoSimulator& simulator_;
    GLFWwindow* window_ = nullptr;

    mjvCamera camera_{};
    mjvOption option_{};
    mjvScene scene_{};
    mjrContext context_{};

    bool paused_ = false;
    bool overhead_ = false;
};
