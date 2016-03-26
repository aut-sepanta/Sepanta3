#!/bin/bash


cmd=""
tab="--tab-with-profile=Default"

#Devices
cmd+=($tab -e "bash -c 'sleep 3;roslaunch openni_launch openni.launch depthregistration:=true'; bash" -t "Kinect")

gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"