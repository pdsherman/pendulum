
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

# Publish single control pulse
add_executable( pulse_test
  pulse_test.cpp )
target_link_libraries( pulse_test
  ${catkin_LIBRARIES} )

add_dependencies( pulse_test ${PROJECT_NAME}_generate_messages )
install(TARGETS pulse_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
