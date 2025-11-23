#!/bin/bash

set -e

source /opt/ros/noetic/setup.bash

if [ -f "/home/${USER}/.bashrc" ]; then
    source /home/${USER}/.bashrc
fi

if [ -f "/home/${USER}/catkin_ws/devel/setup.bash" ]; then
    source /home/${USER}/catkin_ws/devel/setup.bash
fi

if [ $# -eq 0 ]; then
    exec bash
else
    exec "$@"
fi

