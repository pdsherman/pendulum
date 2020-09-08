
# Physics-based simulation
add_executable( simulation
  model_publisher.cpp )
target_link_libraries( simulation
  pendulum_model
  ${catkin_LIBRARIES} )

add_dependencies( simulation ${PROJECT_NAME}_generate_messages )
install(TARGETS simulation RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# File for junk tests
add_executable( main_test misc_test.cpp)
