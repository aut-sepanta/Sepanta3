cmake_minimum_required(VERSION 2.8.3)
project(sepantascenario)

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
  k2_client
)

find_package(Boost  REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN REQUIRED eigen3)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")

catkin_package(
      INCLUDE_DIRS include 
      CATKIN_DEPENDS message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${SERIAL_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(scenario1 src/scenario1.cpp)
target_link_libraries(scenario1 ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(scenario1 sepanta_msgs_generate_messages_cpp)

add_executable(stage1_navigation src/stage1_navigation.cpp)
target_link_libraries(stage1_navigation ${catkin_LIBRARIES} ${Boost_LIBRARIES} )
add_dependencies(stage1_navigation sepanta_msgs_generate_messages_cpp)





