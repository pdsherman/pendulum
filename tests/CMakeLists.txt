
# Super simple state publisher
add_executable( first_pub
  first_publisher.cpp )
target_link_libraries( first_pub
  ${catkin_LIBRARIES} )

add_dependencies( first_pub ${PROJECT_NAME}_generate_messages )
install(TARGETS first_pub RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )

# Physics-based state publisher
add_executable( second_pub
  second_publisher.cpp )
target_link_libraries( second_pub
  ${catkin_LIBRARIES} )

add_dependencies( second_pub ${PROJECT_NAME}_generate_messages )
install(TARGETS second_pub RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
