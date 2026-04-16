#!/bin/bash
# Context: Run this script to start the ROS2 mock robot
# Usage: ./run_mock.sh

# Get the directory where the script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 1. Source ROS2 environment
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
    echo "Sourced ROS2 Humble environment."
else
    echo "Error: ROS2 Humble setup.bash not found at /opt/ros/humble/setup.bash"
    exit 1
fi

# 2. Run the mock robot script
python3 "$SCRIPT_DIR/mock_robot.py"
