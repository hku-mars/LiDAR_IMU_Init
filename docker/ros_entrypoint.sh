#!/bin/bash
 
set -e

# Ros build
source "/opt/ros/melodic/setup.bash"


# Libray install if you want

echo "================Docker Env Ready================"

cd /home/catkin_ws

exec "$@"
