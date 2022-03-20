
# Main Logging Node Executable
add_executable(sql_node
  main.cpp)
target_link_libraries( sql_node
  log_data_hander
  ${catkin_LIBRARIES})

add_dependencies( sql_node ${PROJECT_NAME}_generate_messages )
install(TARGETS sql_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )


# A client test for logging
add_executable( sql_client_test log_client_test.cpp)
target_link_libraries( sql_client_test sql_table func_timer  ${catkin_LIBRARIES} ${CMAKE_THREAD_LIBS_INIT})

add_dependencies( sql_client_test ${PROJECT_NAME}_generate_messages )
install(TARGETS sql_client_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
