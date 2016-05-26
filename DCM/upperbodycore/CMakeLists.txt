cmake_minimum_required(VERSION 2.8.3)
project(upperbodycore)

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
  
)

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


include_directories(include

  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${LIBIK_INCLUDE_DIRS}
  ~/include_lib
)

add_library(ik src/ik/linspace.cpp  src/ik/rtGetInf.cpp  src/ik/rtGetNaN.cpp  src/ik/rt_nonfinite.cpp  src/ik/simple_IK.cpp  src/ik/simple_IK_initialize.cpp  src/ik/simple_IK_terminate.cpp  src/ik/traj_IK_terminate.cpp src/ik/traj_IK_emxAPI.cpp src/ik/traj_IK_emxutil.cpp src/ik/traj_IK.cpp src/ik/traj_IK_initialize.cpp)

target_link_libraries(ik)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS})
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(upperbodycore src/upperbodycore.cpp)
target_link_libraries(upperbodycore ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ik)
add_dependencies(upperbodycore sepanta_msgs_generate_messages_cpp)

add_executable(sepanta_ik src/sepanta_ik.cpp)
target_link_libraries(sepanta_ik ${catkin_LIBRARIES} ${Boost_LIBRARIES} ik )
add_dependencies(sepanta_ik sepanta_msgs_generate_messages_cpp)






