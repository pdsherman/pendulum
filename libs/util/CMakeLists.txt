
add_library(util
  src/util.cpp
  src/ros_util.cpp)
target_link_libraries(util ${catkin_LIBRARIES})
add_dependencies( util ${PROJECT_NAME}_generate_messages )

add_library(func_timer
  src/FunctionTimer.cpp)
