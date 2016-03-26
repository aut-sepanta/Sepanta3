#!/bin/bash


cmd=""
tab="--tab-with-profile=Default"

#Core
cmd+=($tab -e "bash -c 'rosrun athomerobot athomerobot'; bash" -t "Core")
cmd+=($tab -e "bash -c 'rosrun athomerobot odometry'; bash" -t "Odometry")
cmd+=($tab -e "bash -c 'rosrun athomerobot slam'; bash" -t "Slam")
cmd+=($tab -e "bash -c 'rosrun map_server map_server'; bash" -t "Map Server")
cmd+=($tab -e "bash -c 'rosrun athomerobot slam_client'; bash" -t "Slam Client")

gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"