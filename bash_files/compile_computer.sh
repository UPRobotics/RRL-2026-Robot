#!/bin/bash
cd ..
source /opt/ros/humble/setup.bash

rm -rf build install log

colcon build --symlink-install --allow-overriding octomap_msgs
source install/setup.bash
