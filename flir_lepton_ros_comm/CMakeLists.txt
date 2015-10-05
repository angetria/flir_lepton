cmake_minimum_required(VERSION 2.8.3)
project(flir_lepton_ros_comm)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  sensor_msgs
)


add_message_files(
  FILES
  FlirLeptonMsg.msg
)

## Generate added messages with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
catkin_package(
  CATKIN_DEPENDS 
  message_generation
  message_runtime
  std_msgs
  sensor_msgs
)
