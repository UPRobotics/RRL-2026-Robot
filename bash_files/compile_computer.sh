#!/bin/bash
cd ..
source install/setup.bash
source /opt/ros/humble/setup.bash

colcon build --packages-select robot_msgs
colcon build --packages-select telemetry_ui_pkg
colcon build --packages-select control_pkg


