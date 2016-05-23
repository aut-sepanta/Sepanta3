cmake_minimum_required(VERSION 2.8.3)
project(simserver)

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
)



find_package(Boost COMPONENTS system thread date_time REQUIRED)
find_package(PCL 1.2 REQUIRED)
find_package(PkgConfig REQUIRED)
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
)

include_directories(${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS}  ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

add_executable(simserver src/simserver.cpp)
target_link_libraries(simserver  ${catkin_LIBRARIES} ${Boost_LIBRARIES}  ${OpenCV_LIBS}  ${PCL_LIBRARIES})



