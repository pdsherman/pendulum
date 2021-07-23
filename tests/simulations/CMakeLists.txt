

add_executable( control_base_sim
  control_base_sim.cpp )
target_link_libraries( control_base_sim
  control
  pendulum_model
  util
  ${catkin_LIBRARIES} )

add_dependencies( control_base_sim ${PROJECT_NAME}_generate_messages )
install(TARGETS control_base_sim RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )


add_executable( digital_example_sim
  digital_example_sim.cpp )
target_link_libraries( digital_example_sim
  control
  pendulum_model
  util
  ${catkin_LIBRARIES} )

add_dependencies( control_base_sim ${PROJECT_NAME}_generate_messages )
install(TARGETS control_base_sim RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
