

add_library(
  pendulum_model
  src/PendulumModel.cpp)
target_link_libraries(
  pendulum_model
  util
  ${catkin_LIBRARIES} )
add_dependencies( pendulum_model ${PROJECT_NAME}_generate_messages )
