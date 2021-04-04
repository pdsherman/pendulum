
add_executable( sql_test sql_table_test.cpp)
target_link_libraries( sql_test sql_table func_timer)

add_executable( sql_client_test log_client_test.cpp)
target_link_libraries( sql_client_test sql_table func_timer  ${catkin_LIBRARIES})

add_dependencies( sql_client_test ${PROJECT_NAME}_generate_messages )
install(TARGETS sql_client_test RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
