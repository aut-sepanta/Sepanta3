cmake_minimum_required(VERSION 2.8.3)
project(tcpip)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  roscpp
  rospy
  genmsg
  std_msgs
  image_transport
  message_generation
  dynamixel_msgs
  sepanta_msgs
 
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV 2.2 REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(EIGEN REQUIRED eigen3)

catkin_package(
      INCLUDE_DIRS include 
      CATKIN_DEPENDS message_runtime
)

add_library(libtcp
            src/client_tcp.cpp
           )

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(core_tcp src/tcp.cpp)
target_link_libraries(core_tcp ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(core_tcp ${PROJECT_NAME}_generate_messages_cpp sepanta_msgs_generate_messages_cpp)






