
# ROS motor library
add_library(ros_motor src/RosMotor.cpp)
target_link_libraries(ros_motor
  ext_motor
  ${catkin_LIBRARIES})

# Executable
add_executable(ros_motor_node src/motor_node.cpp)
target_link_libraries(ros_motor_node ros_motor)

add_dependencies( ros_motor_node ${PROJECT_NAME}_generate_messages )
install(TARGETS ros_motor_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
