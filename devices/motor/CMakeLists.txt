
add_executable( motor_test
src/main.cpp )
target_link_libraries(motor_test
  motor
  libserial_static
  ${catkin_LIBRARIES})

add_dependencies( motor_test ${PROJECT_NAME}_generate_messages )
install(TARGETS motor_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
