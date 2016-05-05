cmake_minimum_required(VERSION 2.8.3)
project(object_recognition CXX)

find_package(catkin REQUIRED COMPONENTS genmsg actionlib
    actionlib_msgs
    geometry_msgs
    roscpp
    sensor_msgs
    std_msgs
    image_transport
    image_geometry
    cv_bridge
    pcl_conversions
    pcl_ros
    sepanta_msgs
)

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PCL REQUIRED)
pkg_check_modules(EIGEN REQUIRED eigen3)


#add_definitions("-Wno-enam-compare")
add_definitions(${PCL_DEFINITIONS})
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")


################################################
## Declare ROS messages, services and actions ##
################################################

#add_action_files(
#   DIRECTORY action
#   FILES
#logic_grip.action
#object.action
#pickAndPlace.action
#)

#generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs  std_msgs)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs
#  CATKIN_DEPENDS geometry_msgs nav_msgs nodelet roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(include)
include_directories(${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${OMPL_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS}) 

add_executable (train_objects src/train_objects_main.cpp src/train_objects.cpp src/object_pipeline.cpp src/object.cpp)
add_dependencies(train_objects sepanta_msgs_generate_messages_cpp)
target_link_libraries (train_objects ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable (object_recognition src/object_recognition_main.cpp src/object_recognition.cpp src/object_pipeline.cpp src/object.cpp)
add_dependencies(object_recognition sepanta_msgs_generate_messages_cpp)
target_link_libraries(object_recognition ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_executable (visualize_objects src/visualize_objects.cpp)
add_dependencies(visualize_objects sepanta_msgs_generate_messages_cpp)
target_link_libraries(visualize_objects ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES})

add_executable (object_recognition_action_server src/object_recognition_action_server_main.cpp src/object_recognition_action_server.cpp src/object_recognition.cpp src/object_pipeline.cpp src/object.cpp)
add_dependencies(object_recognition_action_server sepanta_msgs_generate_messages_cpp)
target_link_libraries(object_recognition_action_server ${catkin_LIBRARIES} ${OMPL_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})
