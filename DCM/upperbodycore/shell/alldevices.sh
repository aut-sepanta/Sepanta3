#!/bin/bash


cmd=""
tab="--tab-with-profile=Default"

#Devices
#cmd+=($tab -e "bash -c 'sleep 1; roslaunch upperbodycore motor.launch'; bash" -t "Motor")
#cmd+=($tab -e "bash -c 'sleep 5; rosrun upperbodycore upperbodycore'; bash" -t "Core")
#cmd+=($tab -e "bash -c 'sleep 10; rosrun downerbodycore downerbodycore'; bash" -t "Down")
cmd+=($tab -e "bash -c 'sleep 1; rosrun pgitic_play_sound play_sound_core'; bash" -t "Sound")
cmd+=($tab -e "bash -c 'sleep 5; roslaunch skeleton_markers markers.launch'; bash" -t "Markers")
cmd+=($tab -e "bash -c 'sleep 10; rosrun pgitic_wave wave_detect_core'; bash" -t "Wave")
cmd+=($tab -e "bash -c 'sleep 15; rosrun pgitic_skeleton core_skeleton'; bash" -t "Skeleton")
cmd+=($tab -e "bash -c 'sleep 20; rosrun pgitic_senario senario'; bash" -t "Senario")

gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"
