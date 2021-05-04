
add_executable( control_base
  src/control_base.cpp )
target_link_libraries( control_base
  control
  pendulum_model
  util
  ${catkin_LIBRARIES} )

add_dependencies( control_base ${PROJECT_NAME}_generate_messages )
install(TARGETS control_base RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
