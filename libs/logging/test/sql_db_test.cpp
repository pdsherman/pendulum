/*
 * File:   sql_db_test.cpp
 * Author: pdsherman
 * Date:   March 2022
 *
 * Description: Test script for SQLite library
*/

#include <libs/logging/SqliteDatabase.hpp>
#include <libs/logging/SqliteTable.hpp>
#include <libs/util/FunctionTimer.hpp>

#include <iostream>


int main(int argc, char *argv[])
{
  std::shared_ptr<SqliteDatabase> db = std::make_shared<SqliteDatabase>();

  if(db->open_database("test-db-1.db3")){
    std::cout << "Database Connection is Open" << std::endl;

    SqliteTable table(db);
    std::string tbl_name = "Table2";
    std::vector<std::string> cols({"TEST_STR", "A", "B", "D"});

    if(table.initialize_table(tbl_name, cols)) {
      std::cout << "Table is Ready"  << std::endl;
      std::string tst = "sql_db_test";
      std::vector<double> data(3);

      util::FunctionTimer tmr;
      for(int ii = 0; ii < 10000; ++ii) {
        data[0] = ii;
        data[1] = ii * 3;
        data[2] = ii * ii;

        tmr.start();
        table.insert_row(tst, data);
        tmr.stop();
      }
      std::cout << "Time: " << tmr.average_us()/1000.0 << " ms" << std::endl;
    }
  }

  std::cout << "...End of Test" << std::endl;

  return 0;
}
