#!/bin/bash

rosnode kill /camera/camera_nodelet_manager
rosnode kill /camera/debayer
rosnode kill /camera/depth_metric
rosnode kill /camera/depth_metric_rect
rosnode kill /camera/depth_points
rosnode kill /camera/depth_rectify_depth
rosnode kill /camera/depth_registered_hw_metric_rect
rosnode kill /camera/depth_registered_metric
rosnode kill /camera/depth_registered_rectify_depth
rosnode kill /camera/depth_registered_sw_metric_rect
rosnode kill /camera/disparity_depth
rosnode kill /camera/disparity_registered_hw
rosnode kill /camera/disparity_registered_sw
rosnode kill /camera/driver
rosnode kill /camera/points_xyzrgb_hw_registered
rosnode kill /camera/points_xyzrgb_sw_registered
rosnode kill /camera/rectify_color
rosnode kill /camera/rectify_ir
rosnode kill /camera/rectify_mono
rosnode kill /camera/register_depth_rgb
rosnode kill /camera_base_link
rosnode kill /camera_base_link1
rosnode kill /camera_base_link2
rosnode kill /camera_base_link3

killall nodelet
rosnode cleanup /camera/camera_nodelet_manager






