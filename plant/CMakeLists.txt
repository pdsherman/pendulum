
add_library( pendulum_model
  src/Model.cpp )
target_link_libraries( pendulum_model
  ${catkin_LIBRARIES} )

add_library( pendulum_simple_model
src/SimpleModel.cpp )
target_link_libraries( pendulum_simple_model
  ${catkin_LIBRARIES} )

add_library( dirt_simple_model
src/SimplerModel.cpp )
target_link_libraries( dirt_simple_model
    ${catkin_LIBRARIES} )

add_dependencies( pendulum_model ${PROJECT_NAME}_generate_messages )
add_dependencies( pendulum_simple_model ${PROJECT_NAME}_generate_messages )
add_dependencies( dirt_simple_model ${PROJECT_NAME}_generate_messages )
