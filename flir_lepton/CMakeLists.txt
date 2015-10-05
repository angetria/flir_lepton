cmake_minimum_required(VERSION 2.8.3)
project(flir_lepton)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  flir_lepton_ros_comm
)


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    message_runtime
    roscpp
    sensor_msgs
    flir_lepton_ros_comm
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
  src/utils.cpp
)

add_library(${PROJECT_NAME}_lib
  src/flir_lepton.cpp
)

target_link_libraries(${PROJECT_NAME}_lib
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_utils_lib
)

################################################################################
#                  Flir Lepton Hardware Interface Executable                   #
################################################################################

add_executable(${PROJECT_NAME}_node
  src/flir_lepton_node.cpp
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  ${PROJECT_NAME}_lib
)
