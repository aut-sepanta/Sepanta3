#!/bin/bash


cmd=""
tab="--tab-with-profile=Default"

#Map

cmd+=($tab -e "bash -c 'rosrun fetch_and_carry object_party'; bash" -t "object")
cmd+=($tab -e "bash -c 'rosrun fetch_and_carry grip_party'; bash" -t "grip")
cmd+=($tab -e "bash -c 'rosrun cocktail_party cocktail_party_logic'; bash" -t "coc")

gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"