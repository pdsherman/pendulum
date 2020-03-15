
add_library( control
  src/ControlBase.cpp
  src/Pid.cpp )

add_executable( control_loop
  src/control_loop.cpp )
target_link_libraries( control_loop
  control
  ${catkin_LIBRARIES} )

add_dependencies( control_loop ${PROJECT_NAME}_generate_messages )
install(TARGETS control_loop RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
