cmake_minimum_required(VERSION 2.8.3)
project(flir_lepton_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  std_msgs
  sensor_msgs
)


add_message_files(DIRECTORY msg/flir_lepton_sensor
  FILES
  TemperaturesMsg.msg
  FlirLeptonBatchMsg.msg
  FlirLeptonRawMsg.msg
)


add_message_files(DIRECTORY msg/flir_lepton_image_processing
  FILES
  CandidateRoiMsg.msg
  CandidateRoisVectorMsg.msg
  GeneralAlertInfo.msg
  ThermalAlert.msg
  ThermalAlertVector.msg
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
  std_msgs
  sensor_msgs
)
