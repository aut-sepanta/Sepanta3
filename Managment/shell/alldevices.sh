#!/bin/bash


cmd=""
tab="--tab-with-profile=Default"

#Devices
cmd+=($tab -e "bash -c 'sleep 1; roslaunch athomerobot motor.launch'; bash" -t "Dynamixel")
cmd+=($tab -e "bash -c 'sleep 2; roslaunch athomerobot laser.launch'; bash" -t "Laser")
cmd+=($tab -e "bash -c 'sleep 3; roslaunch openni_launch openni.launch depthregistration:=true'; bash" -t "Kinect")

gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"