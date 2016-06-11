cmake_minimum_required(VERSION 2.8.3)
project(SepantaFollow)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  genmsg
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
      CATKIN_DEPENDS actionlib_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${SERIAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(SepantaFollow src/sepantafollowengine_node.cpp src/sepantafollowengine.cpp)
target_link_libraries(SepantaFollow ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(SepantaFollow sepanta_msgs_generate_messages_cpp)


