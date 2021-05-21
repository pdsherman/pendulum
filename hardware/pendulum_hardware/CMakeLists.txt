
# Hardware Library
add_library(pendulum_hardware src/PendulumHardware.cpp)
target_link_libraries(pendulum_hardware
  ext_motor
  encoder
  ${catkin_LIBRARIES})

# Executable
add_executable(pendulum_hardware_node src/main.cpp)
target_link_libraries(pendulum_hardware_node
  pendulum_hardware
  func_timer
  util)

add_dependencies( pendulum_hardware_node ${PROJECT_NAME}_generate_messages )
install(TARGETS pendulum_hardware_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
