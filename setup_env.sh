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

# 2. Add local and system library paths to LD_LIBRARY_PATH
# This ensures custom messages, plugins, and ROS2 dependencies are found at runtime
LOCAL_LIB_PATH="$PROJECT_ROOT/build/lib"
ROS_LIB_PATH="/opt/ros/humble/lib"

export LD_LIBRARY_PATH="$LOCAL_LIB_PATH:$ROS_LIB_PATH:$LD_LIBRARY_PATH"
echo "Updated LD_LIBRARY_PATH to include local and ROS2 libraries."

# 3. Export ROS environment variables for consistency
export AMENT_PREFIX_PATH="/opt/ros/humble:$AMENT_PREFIX_PATH"

echo "Environment setup complete."
