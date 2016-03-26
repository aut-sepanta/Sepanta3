#!/bin/bash


cmd=""
tab="--tab-with-profile=Default"

#BF Nodes
cmd+=($tab -e "bash -c 'rosrun pick_and_place build_pcd_list'; bash" -t "PCD List")
cmd+=($tab -e "bash -c 'rosrun what_did_you_say what_did_you_say_logic'; bash" -t "Logic WDYS")
cmd+=($tab -e "bash -c 'rosrun pick_and_place logic_pick_place'; bash" -t "Logic PP")
cmd+=($tab -e "bash -c 'rosrun avoidthat avoidthat'; bash" -t "Logic AT")
cmd+=($tab -e "bash -c 'rosrun pick_and_place grip_pick_place'; bash" -t "Grip")
cmd+=($tab -e "bash -c 'sleep 5; rosrun pick_and_place object_recognition_shotcolor'; bash" -t "Object")
cmd+=($tab -e "bash -c 'rosrun what_did_you_say find_person'; bash" -t "Find Person")
cmd+=($tab -e "bash -c 'rosrun basic_function client_stop'; bash" -t "Gesture Stop")
cmd+=($tab -e "bash -c 'rosrun athomerobot communication'; bash" -t "Windows")
cmd+=($tab -e "bash -c 'rosrun athomerobot gesture'; bash" -t "Gesture")
cmd+=($tab -e "bash -c 'rosrun basic_function basicFunction'; bash" -t "Task BF")

gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"