#!/bin/bash
roslaunch dynamixel_tutorials controller_spawner.launch &
roslaunch dynamixel_tutorials controller_manager.launch &
rosrun upperbodycore dummy


