#!/bin/bash
cmd=""
tab="--tab-with-profile=Default"

#Devices
cmd+=($tab -e "bash -c 'sleep 1; rosnode kill /camera_base_link'; bash" -t "1")
cmd+=($tab -e "bash -c 'sleep 1; rosnode kill /camera_base_link1'; bash" -t "2")
cmd+=($tab -e "bash -c 'sleep 1; rosnode kill /camera_base_link2'; bash" -t "3")
cmd+=($tab -e "bash -c 'sleep 1; rosnode kill /camera_base_link3'; bash" -t "4")
cmd+=($tab -e "bash -c 'sleep 2; rosnode kill /camera/camera_nodelet_manager'; bash" -t "5")
cmd+=($tab -e "bash -c 'sleep 2; rosnode kill /camera/debayer'; bash" -t "6")
cmd+=($tab -e "bash -c 'sleep 2; rosnode kill /camera/driver'; bash" -t "7")

cmd+=($tab -e "bash -c 'sleep 3; rosnode kill /camera/depth_metric'; bash" -t "8")
cmd+=($tab -e "bash -c 'sleep 3; rosnode kill /camera/depth_metric_rect'; bash" -t "9")
cmd+=($tab -e "bash -c 'sleep 3; rosnode kill /camera/depth_points'; bash" -t "10")
cmd+=($tab -e "bash -c 'sleep 3; rosnode kill /camera/depth_rectify_depth'; bash" -t "11")
cmd+=($tab -e "bash -c 'sleep 3; rosnode kill /camera/depth_registered_rectify_depth'; bash" -t "12")
cmd+=($tab -e "bash -c 'sleep 3; rosnode kill /camera/disparity_depth'; bash" -t "13")
cmd+=($tab -e "bash -c 'sleep 3; rosnode kill /camera/disparity_registered_hw'; bash" -t "14")
cmd+=($tab -e "bash -c 'sleep 3; rosnode kill /camera/disparity_registered_sw'; bash" -t "15")

cmd+=($tab -e "bash -c 'sleep 4; rosnode kill /camera/register_depth_rgb'; bash" -t "16")
cmd+=($tab -e "bash -c 'sleep 4; rosnode kill /camera/rectify_mono'; bash" -t "17")
cmd+=($tab -e "bash -c 'sleep 4; rosnode kill /camera/rectify_ir'; bash" -t "18")
cmd+=($tab -e "bash -c 'sleep 4; rosnode kill /camera/rectify_color'; bash" -t "19")
cmd+=($tab -e "bash -c 'sleep 4; rosnode kill /camera/points_xyzrgb_sw_registered'; bash" -t "20")
cmd+=($tab -e "bash -c 'sleep 4; rosnode kill /camera/points_xyzrgb_hw_registered'; bash" -t "21")

gnome-terminal --working-directory ~/catkin_ws "${cmd[@]}"

