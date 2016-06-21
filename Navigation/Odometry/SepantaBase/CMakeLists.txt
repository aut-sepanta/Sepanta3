cmake_minimum_required(VERSION 2.8.3)
project(odometry_base)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  genmsg
  pcl_conversions
  pcl_ros
  tf
  std_msgs
  nav_msgs
  geometry_msgs
  image_transport
  message_generation
  actionlib
  actionlib_msgs 
  dynamixel_msgs
  sepanta_msgs
)

find_package(Boost  REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN REQUIRED eigen3)

catkin_package(
      INCLUDE_DIRS include 
      CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${SERIAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})



add_executable(odometry src/odometry.cpp)
target_link_libraries(odometry ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(odometry sepanta_msgs_generate_messages_cpp)

add_executable(client_odometry src/client_odometry.cpp)
target_link_libraries(client_odometry ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(client_odometry sepanta_msgs_generate_messages_cpp)

add_executable(cmd_watchdog src/cmd_watchdog.cpp)
target_link_libraries(cmd_watchdog ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(cmd_watchdog sepanta_msgs_generate_messages_cpp)








