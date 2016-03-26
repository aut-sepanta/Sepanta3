cmake_minimum_required(VERSION 2.8.3)
project(sound)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  message_generation
  actionlib
  actionlib_msgs
  sepanta_msgs
)

## Find catkin and any catkin packages

find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)
pkg_check_modules(EIGEN REQUIRED eigen3)

add_library(soundclient_lib
 src/soundclient.cpp
 )

 
target_link_libraries(soundclient_lib ${OpenCV_LIBS} ${catkin_LIBRARIES})
add_library(pgiticlib_sound
 src/client_sound.cpp
 )
 
 target_link_libraries(pgiticlib_sound ${OpenCV_LIBS} ${catkin_LIBRARIES})

catkin_package(
      INCLUDE_DIRS include 
      CATKIN_DEPENDS message_runtime
)


add_dependencies(soundclient_lib  sepanta_msgs_generate_messages_cpp)
add_dependencies(pgiticlib_sound  sepanta_msgs_generate_messages_cpp)

add_executable(core_sound src/core_sound.cpp)
target_link_libraries(core_sound  ${catkin_LIBRARIES} soundclient_lib)
add_dependencies(core_sound  sepanta_msgs_generate_messages_cpp)

add_executable(sample_sound src/sample_sound.cpp)
target_link_libraries(sample_sound  ${catkin_LIBRARIES} pgiticlib_sound)
add_dependencies(sample_sound  sepanta_msgs_generate_messages_cpp)

add_executable(sampleLogRead src/sampleLogRead.cpp)
target_link_libraries(sampleLogRead  ${catkin_LIBRARIES} pgiticlib_sound)
add_dependencies(sampleLogRead  sepanta_msgs_generate_messages_cpp)


#add_executable(speech src/speech.cpp)
#target_link_libraries(speech ${catkin_LIBRARIES})
#add_dependencies(speech athomerobot_msgs_generate_messages_cpp)

# %EndTag(FULLTEXT)%
