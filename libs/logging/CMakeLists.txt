
add_library( sql_table
  src/SqliteTable.cpp )
target_link_libraries( sql_table
  func_timer
  ${SQLITE3_LIBRARIES} )
add_dependencies( sql_table ${PROJECT_NAME}_generate_messages )

add_executable( sql_node
  src/main.cpp
  src/MsgHandler.cpp)
target_link_libraries( sql_node
  sql_table
  ${catkin_LIBRARIES})

add_dependencies( sql_node ${PROJECT_NAME}_generate_messages )
install(TARGETS sql_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} )
