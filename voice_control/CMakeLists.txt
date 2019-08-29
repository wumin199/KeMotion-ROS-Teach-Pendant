cmake_minimum_required(VERSION 2.8.3)
project(voice_control)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


#add_executable(iat_voice_control
#  src/iat_online_record_sample.cpp
#  src/linuxrec.c
#  src/speech_recognizer.c
#)
#target_link_libraries(iat_voice_control ${catkin_LIBRARIES} libmsc.so -lmsc -lrt -ldl -lpthread -lasound -lstdc++)


add_executable(arm_voice_control
  src/iat_voice_control.cpp
  src/linuxrec.c
  src/speech_recognizer.c
)
target_link_libraries(arm_voice_control ${catkin_LIBRARIES} libmsc.so -lmsc -lrt -ldl -lpthread -lasound -lstdc++)
