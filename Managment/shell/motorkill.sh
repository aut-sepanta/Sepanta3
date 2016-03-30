#!/bin/bash
rosnode kill /dynamixel_controller_spawner
rosnode kill /dynamixel_manager
killall python
rosnode kill /dummy
killall dummy
