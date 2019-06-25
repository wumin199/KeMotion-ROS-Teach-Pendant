cmake_minimum_required(VERSION 2.8.3)
project(rtp_gui)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  std_msgs
  rtp_msgs
  std_srvs
  geometry_msgs
  genmsg
  moveit_core
  moveit_planners_ompl
  moveit_ros_planning
  moveit_ros_planning_interface
  tf
  tf_conversions
  actionlib
  control_msgs
  robot_movement_interface
)
#find_package(Boost REQUIRED COMPONENTS system thread)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
#set(CMAKE_AUTORCC ON)
set(CMAKE_INCLUDE_CURRENT_DIR ON)


find_package(Qt5  REQUIRED COMPONENTS Core Gui Widgets)
qt5_wrap_cpp(MOC src/gui/mainwindow.h)
qt5_wrap_ui(UIC src/gui/mainwindow.ui)

set(SOURCES
  src/gui/mainwindow.cpp
  src/gui/main.cpp
  src/nodes/rtp_data_type.cpp
  #src/gui/mainwindow.h
)

set(FORMS
  src/gui/mainwindow.ui
)


generate_dynamic_reconfigure_options(
  cfg/RtpDynamicReconfigure.cfg
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES rtp_gui
  CATKIN_DEPENDS roscpp std_msgs moveit_core moveit_ros_planning moveit_ros_planning_interface moveit_planners_ompl tf tf_conversions robot_movement_interface
  #DEPENDS Boost
)

include_directories(
  include
  ${CMAKE_CURRENT_BINARY_DIR}/.. ##非常重要，用于找到.ui文件转换后的.h，否则即使编译成功，也检索不到ui_mainwindow.h
  ${catkin_INCLUDE_DIRS}
)


add_executable(rtp_gui_node ${SOURCES} ${FORMS} )

target_link_libraries(rtp_gui_node  ${catkin_LIBRARIES} Qt5::Widgets) #without Qt5::Widgets IDE will not hightlight Qt library
add_dependencies(rtp_gui_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS}) #msg,srv etc


add_executable(rtp_node
  src/nodes/main.cpp
  src/nodes/rtp_teleop.cpp
  src/nodes/rtp_manager.cpp
)

target_link_libraries(rtp_node ${catkin_LIBRARIES} Qt5::Widgets) #without Qt5::Widgets IDE will not hightlight Qt library
add_dependencies(rtp_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})  #msg,srv etc

