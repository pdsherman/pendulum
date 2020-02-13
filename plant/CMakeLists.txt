
add_library( pendulum_model
  src/Model.cpp )
target_link_libraries( pendulum_model
  ${catkin_LIBRARIES} )

add_dependencies( pendulum_model ${PROJECT_NAME}_generate_messages )
