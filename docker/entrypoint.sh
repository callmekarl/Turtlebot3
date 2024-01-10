#!/bin/bash

# Source ROS2 environment

cd /Turtlebot3_ws/
source /opt/ros/humble/setup.bash

# Source your workspace setup script, replace '~/turtlebot3_ws' with your workspace path
source install/setup.bash

# Execute the passed command
exec "$@"
