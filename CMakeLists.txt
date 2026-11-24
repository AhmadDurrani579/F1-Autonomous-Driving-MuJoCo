
cmake_minimum_required(VERSION 3.20)

project(F1MuJoCo LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Apple Silicon Homebrew location
list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")

# MuJoCo framework
set(MUJOCO_FRAMEWORK
    "$ENV{HOME}/Library/Frameworks/mujoco.framework"
)

if(NOT EXISTS "${MUJOCO_FRAMEWORK}/Headers/mujoco.h")
    message(FATAL_ERROR "MuJoCo header not found")
endif()

# Find GLFW
find_package(glfw3 CONFIG REQUIRED)

# Main executable
add_executable(f1_sim
    src/main.cpp
    src/simulation/MuJoCoSimulator.cpp
    src/rendering/MuJoCoViewer.cpp
    src/controllers/PurePursuit.cpp
)

# Header directories
target_include_directories(f1_sim PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/third_party/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/src"
)

# Link MuJoCo and GLFW
target_link_libraries(f1_sim PRIVATE
    "${MUJOCO_FRAMEWORK}"
    glfw
)

# Runtime library location
set_target_properties(f1_sim PROPERTIES
    BUILD_RPATH "$ENV{HOME}/Library/Frameworks"
)
