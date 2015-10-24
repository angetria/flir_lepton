cmake_minimum_required(VERSION 2.8.3)
project(flir_lepton_image_processing)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  #image_transport
  dynamic_reconfigure
  flir_lepton_msgs
  roslint
  )

generate_dynamic_reconfigure_options(
  cfg/thermal_cfg.cfg
  )

catkin_package(
  CATKIN_DEPENDS
    roscpp
    cv_bridge
    #image_transport
    dynamic_reconfigure
    flir_lepton_msgs
  INCLUDE_DIRS
    include
  LIBRARIES
  )

include_directories(
  include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
  )

################################ Utils library #################################
add_library(${PROJECT_NAME}_utils
  src/utils/blob_detection.cpp
  src/utils/bounding_box_detection.cpp
  src/utils/edge_detection.cpp
  src/utils/roi_filters.cpp
  src/utils/thermal_rois_conveyor.cpp
  src/utils/message_conversions.cpp
  src/utils/morphological_operators.cpp
  src/utils/outline_discovery.cpp
  src/utils/parameters.cpp
  src/utils/timer.cpp
  src/utils/visualization.cpp
  )

# Ensure all their headers get built before any targets that need them
add_dependencies(${PROJECT_NAME}_utils ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}_utils
  ${catkin_LIBRARIES}
  )

add_subdirectory(src/processing_node)

################################# ROSLINT ######################################
# Execute via lint : catkin_make roslint_{PACKAGE_NAME}

set(ROSLINT_CPP_OPTS
  "--filter=-whitespace/end_of_line, -build/include_order, -whitespace/blank_line, -whitespace/parens, -whitespace/comments, -build/include_what_you_use, -whitespace/braces, -whitespace/newline, -readability/multiline_string, -runtime/printf, -readability/streams")

set(ROSLINT_CPP_OPTS
  "--filter=-build/include_what_you_use, -runtime/references")

file(GLOB_RECURSE ${PROJECT_NAME}_LINT_SRCS
  RELATIVE ${PROJECT_SOURCE_DIR}

  include/*.h
  src/*.cpp
  )

roslint_cpp(${${PROJECT_NAME}_LINT_SRCS})
