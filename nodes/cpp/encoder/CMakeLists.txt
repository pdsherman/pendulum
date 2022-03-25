
# Node Executable For Single Encoder ROS nose
add_executable(single_encoder_node
  single_encoder.cpp)
target_link_libraries(single_encoder_node
  encoder
  util
  ${catkin_LIBRARIES})

add_dependencies( single_encoder_node ${PROJECT_NAME}_generate_messages )
install(TARGETS single_encoder_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
