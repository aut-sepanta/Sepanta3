#!/bin/bash


cmd=""
tab="--tab-with-profile=Default"

#Map

cmd+=($tab -e "bash -c 'roslaunch athomerobot hector_slam.launch'; bash" -t "Map")
#cmd+=($tab -e "bash -c 'sleep 3;rosrun athomerobot hectoroffset'; bash" -t "Map")
cmd+=($tab -e "bash -c 'sleep 4;roslaunch athomerobot move_base.launch'; bash" -t "Move")

gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"