
add_library( encoder
  src/Encoder.cpp )

add_executable( encoder_test
src/encoder_main.cpp )
target_link_libraries( encoder_test
  encoder
  ${catkin_LIBRARIES} )

add_dependencies( encoder_test ${PROJECT_NAME}_generate_messages )
install(TARGETS encoder_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
