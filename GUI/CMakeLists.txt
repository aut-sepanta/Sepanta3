cmake_minimum_required(VERSION 2.8.3)

project(rqt_camera_calib_gui)
# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge )

find_package(Qt4 COMPONENTS QtCore QtGui REQUIRED)

include(${QT_USE_FILE})

set(rqt_camera_calib_gui_SRCS
  src/rqt_camera_calib_gui/CameraCalibView.cpp
)

set(rqt_camera_calib_gui_HDRS
  include/rqt_camera_calib_gui/CameraCalibView.h
)

set(rqt_camera_calib_gui_UIS
  resources/camera_calib_view.ui
)

set(rqt_camera_calib_gui_INCLUDE_DIRECTORIES
  include
  ${CMAKE_CURRENT_BINARY_DIR}
  $ENV{AUTMAN_BASE_DIR}
)

catkin_package(
  INCLUDE_DIRS ${rqt_camera_calib_gui_INCLUDE_DIRECTORIES}
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS rqt_gui rqt_gui_cpp image_transport sensor_msgs cv_bridge sepanta_msgs 
)
catkin_python_setup()

qt4_wrap_cpp(rqt_camera_calib_gui_MOCS ${rqt_camera_calib_gui_HDRS})
qt4_wrap_ui(rqt_camera_calib_gui_UIS_H ${rqt_camera_calib_gui_UIS})

include_directories(${rqt_camera_calib_gui_INCLUDE_DIRECTORIES} ${catkin_INCLUDE_DIRS}) 
add_library(${PROJECT_NAME} ${rqt_camera_calib_gui_SRCS} ${rqt_camera_calib_gui_MOCS} ${rqt_camera_calib_gui_UIS_H})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${QT_QTCORE_LIBRARY} ${QT_QTGUI_LIBRARY})
add_dependencies(${PROJECT_NAME} sepanta_msgs_generate_messages_cpp)

find_package(class_loader)
class_loader_hide_library_symbols(${PROJECT_NAME})

install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(PROGRAMS scripts/rqt_camera_calib_gui
  DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)
install(PROGRAMS scripts/rqt_camera_calib_gui
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
