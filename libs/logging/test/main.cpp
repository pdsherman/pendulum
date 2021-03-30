/*
 * File:   main.cpp
 * Author: pdsherman
 * Date:   March 2021
 *
 * Description: Test script for SQLite library
*/

#include <libs/logging/SqliteTable.hpp>
#include <libs/function_timer/FunctionTimer.hpp>


#include <string>
#include <vector>
#include <iostream>

static const std::string db_filename = "TestDatabase.db";

int main(int argc, char* argv[])
{
  SqliteTable table("TestTable");
  if(table.open_database(db_filename)){
    std::vector<std::string> cols({"ColIndex", "A", "B", "C", "D"});
    table.create_table(cols);


    util::FunctionTimer tmr;

    for(size_t i = 0; i < 10000; ++i) {
      tmr.start();
      table.insert_row("Trial_01", {3.3+i, 4.3, 5.3, 7.3});
      tmr.stop();
    }
    table.delete_table();

    std::cout << "Time: " << tmr.average_us()/1000.0 << " ms" << std::endl;

  } else {
    std::cout << "Unable to open database" << std::endl;
  }

  return 0;
}
