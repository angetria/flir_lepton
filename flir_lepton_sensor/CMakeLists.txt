cmake_minimum_required(VERSION 2.8.3)
project(flir_lepton_sensor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  roslib
  sensor_msgs
  flir_lepton_msgs
)


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    message_runtime
    roscpp
    roslib
    sensor_msgs
    flir_lepton_msgs
  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


################################################################################
#                      Flir Lepton Hardware Interface Lib                      #
################################################################################

add_library(${PROJECT_NAME}_utils_lib
  src/flir_lepton_utils.cpp
)

add_library(${PROJECT_NAME}_lib
  src/flir_lepton_hw_iface.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils_lib
)

add_dependencies(${PROJECT_NAME}_lib ${catkin_EXPORTED_TARGETS})


################################################################################
#                  Flir Lepton Sensor Executable                               #
################################################################################

add_executable(${PROJECT_NAME}_node
  src/flir_lepton_sensor.cpp
)

add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)
