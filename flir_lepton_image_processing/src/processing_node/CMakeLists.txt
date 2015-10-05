add_library(${PROJECT_NAME}_process
  process.cpp
  thermal_roi_detector.cpp
  )

target_link_libraries(${PROJECT_NAME}_process
  ${PROJECT_NAME}_utils
  ${catkin_LIBRARIES})

add_dependencies(${PROJECT_NAME}_process
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS})

add_executable(thermal_camera_node process_node.cpp)

target_link_libraries(thermal_camera_node
  ${PROJECT_NAME}_process
  ${catkin_LIBRARIES})
