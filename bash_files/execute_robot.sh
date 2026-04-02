#!/bin/bash
cd ..
source /opt/ros/humble/setup.bash

if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: No se encontró la carpeta install. ¿Ya compilaste?"
    exit 1
fi

ros2 launch robot_pkg robot.launch.py