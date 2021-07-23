

add_executable( control_base
  control_base.cpp )
target_link_libraries( control_base
  control
  pendulum_hardware
  util
  ${Boost_PROGRAM_OPTIONS_LIBRARY}
  ${catkin_LIBRARIES} )

add_dependencies( control_base ${PROJECT_NAME}_generate_messages )
install(TARGETS control_base RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
