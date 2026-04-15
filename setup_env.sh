#!/bin/bash
# Context: Source this script to set up the environment for Ros_Qt5_Gui_App
# Usage: source setup_env.sh

# Get the directory where the script is located
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "Sourced ROS2 Humble environment."
else
    echo "Warning: ROS2 Humble setup.bash not found at /opt/ros/humble/setup.bash"
fi

# 2. Add local library path (build/lib) to LD_LIBRARY_PATH
# This ensures custom messages and plugins are found at runtime
LOCAL_LIB_PATH="$PROJECT_ROOT/build/lib"
if [ -d "$LOCAL_LIB_PATH" ]; then
    export LD_LIBRARY_PATH="$LOCAL_LIB_PATH:$LD_LIBRARY_PATH"
    echo "Added $LOCAL_LIB_PATH to LD_LIBRARY_PATH."
else
    echo "Note: Local lib directory $LOCAL_LIB_PATH not found. Run build first."
fi

echo "Environment setup complete."
