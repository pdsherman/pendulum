
# Physics-based simulation
add_executable( parameter_search
  find_parameters.cpp )
target_link_libraries( parameter_search
  pendulum_model
  util
  ${catkin_LIBRARIES} )

add_dependencies( parameter_search ${PROJECT_NAME}_generate_messages )
install(TARGETS parameter_search RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
