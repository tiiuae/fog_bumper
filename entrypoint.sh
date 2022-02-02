#!/bin/bash

source /opt/ros/galactic/setup.bash

ROS_FLAGS=""
if [ ${SIMULATION+x} != "" ]; then
    ROS_FLAGS="use_sim_time:=true ${ROS_FLAGS}"
fi

ros2 launch fog_bumper bumper_launch.py ${ROS_FLAGS}
