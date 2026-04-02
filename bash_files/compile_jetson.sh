#!/bin/bash
cd ..
source install/setup.bash
source /opt/ros/humble/setup.bash

rm -rf build

colcon build --packages-select robot_msgs
colcon build --packages-select robot_pkg
colcon build --packages-select telemetry_pkg


