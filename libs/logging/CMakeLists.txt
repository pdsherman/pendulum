
add_library( sql_table
  src/SqliteTable.cpp
  src/SqliteDatabase.cpp)
target_link_libraries( sql_table ${SQLITE3_LIBRARIES} )

add_library( log_data_hander
  src/DataHandler.cpp)
target_link_libraries( log_data_hander sql_table )

add_subdirectory(test)
