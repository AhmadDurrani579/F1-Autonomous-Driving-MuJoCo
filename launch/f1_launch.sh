#!/bin/bash
set -euo pipefail

# ==========================================
# F1 MUJOCO SIMULATION LAUNCHER
# ==========================================

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

# Load simulation configuration
source "$ROOT/config/simulation.env"

# Select controller
SELECTED_CONTROLLER="${1:-$CONTROLLER}"

# MuJoCo Studio executable
STUDIO="/Applications/MuJoCoStudio.app/Contents/MacOS/mujoco_studio"

# Absolute model path
MODEL_PATH="$ROOT/$MODEL"

echo "========================================="
echo "       F1 MUJOCO SIMULATION"
echo "========================================="
echo "Vehicle:      $VEHICLE"
echo "Track model:  $MODEL"
echo "Waypoints:    $WAYPOINTS"
echo "Controller:   $SELECTED_CONTROLLER"
echo "Target speed: $TARGET_SPEED m/s"
echo "Maximum:      $MAX_SPEED m/s"
echo "Timestep:     $TIMESTEP s"
echo "LiDAR:        $LIDAR_ENABLED"
echo "Camera:       $CAMERA_ENABLED"
echo "========================================="

# Verify required files
for path in "$MODEL" "$VEHICLE" "$WAYPOINTS"; do
    if [[ ! -f "$path" ]]; then
        echo "ERROR: Missing file: $path"
        exit 1
    fi
done

# Verify MuJoCo Studio executable
if [[ ! -x "$STUDIO" ]]; then
    echo "ERROR: MuJoCo Studio executable not found."
    exit 1
fi

echo "All required files verified."
echo ""
echo "Launching Formula 1 car and racetrack..."
echo "Model: $MODEL_PATH"
echo ""

# Direct launch with XML model
exec "$STUDIO" "$MODEL_PATH"
