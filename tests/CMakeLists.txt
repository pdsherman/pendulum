
# Super simple state publisher
add_executable( first_pub
  first_publisher.cpp )
target_link_libraries( first_pub
  ${catkin_LIBRARIES} )

add_dependencies( first_pub ${PROJECT_NAME}_generate_messages )
install(TARGETS first_pub RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Physics-based simulation
add_executable( simulation
  model_publisher.cpp )
target_link_libraries( simulation
  pendulum_model
  ${catkin_LIBRARIES} )

add_dependencies( simulation ${PROJECT_NAME}_generate_messages )
install(TARGETS simulation RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )


# Physics-based simulation of simple model
add_executable( simple_simulation
  simple_model_publisher.cpp )
target_link_libraries( simple_simulation
  pendulum_simple_model
  util
  ${catkin_LIBRARIES} )

add_dependencies( simple_simulation ${PROJECT_NAME}_generate_messages )
install(TARGETS simple_simulation RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )


# Physics-based simulation of simple model
add_executable( dirt_simple_simulation
dirt_simple_model.cpp )
target_link_libraries( dirt_simple_simulation
  dirt_simple_model
  util
  ${catkin_LIBRARIES} )

add_dependencies( simple_simulation ${PROJECT_NAME}_generate_messages )
install(TARGETS simple_simulation RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Publish single control pulse
add_executable( pulse_test
  pulse_test.cpp )
target_link_libraries( pulse_test
  ${catkin_LIBRARIES} )

add_dependencies( pulse_test ${PROJECT_NAME}_generate_messages )
install(TARGETS pulse_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
