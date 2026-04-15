#!/bin/bash
# Source this file to configure ROS2 for local roboticaWS development
# Usage: source setup_local.sh

source /opt/ros/humble/setup.bash
source /home/chumbi/roboticaWS/install/setup.bash

export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
unset CYCLONEDDS_URI

echo "Configured for local roboticaWS development (default CycloneDDS)"
