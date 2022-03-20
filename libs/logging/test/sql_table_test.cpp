/*
 * File:   main.cpp
 * Author: pdsherman
 * Date:   March 2021
 *
 * Description: Test script for SQLite library
*/

#include <libs/logging/SqliteTable.hpp>
#include <libs/util/FunctionTimer.hpp>


#include <string>
#include <vector>
#include <iostream>
#include <atomic>
#include <thread>
#include <random>

static const std::string db_filename = "TestDatabase.db";

void write_to_table(std::unique_ptr<SqliteTable> tbl)
{
  std::string tst = "thread_test";
  std::vector<double> data(3);

  for(int ii = 0; ii < 10000; ++ii) {
    data[0] = ii;
    data[1] = ii * 3;
    data[2] = ii * ii;
    tbl->insert_row(tst, data);
  }
}

int main(int argc, char* argv[])
{
  std::shared_ptr<SqliteDatabase> db = std::make_shared<SqliteDatabase>();
  if(db->open_database("test-db-1.db3")){
    std::unique_ptr<SqliteTable> tbl1 = std::unique_ptr<SqliteTable>(new  SqliteTable(db));
    std::unique_ptr<SqliteTable> tbl2 = std::unique_ptr<SqliteTable>(new  SqliteTable(db));
    std::unique_ptr<SqliteTable> tbl3 = std::unique_ptr<SqliteTable>(new  SqliteTable(db));

    std::string tbl_name1 = "Table01";
    std::string tbl_name2 = "Table02";
    std::string tbl_name3 = "Table03";
    std::vector<std::string> cols({"TEST_STR", "A", "B", "C"});

    if(tbl1->initialize_table(tbl_name1, cols) &&
        tbl2->initialize_table(tbl_name2, cols) &&
        tbl3->initialize_table(tbl_name3, cols)) {

          // Create three threads
          // Insert a bunch of data at the same time
          std::thread t1 = std::thread(write_to_table, std::move(tbl1));
          std::thread t2 = std::thread(write_to_table, std::move(tbl2));
          std::thread t3 = std::thread(write_to_table, std::move(tbl3));

          if(t1.joinable())
            t1.join();
          if(t2.joinable())
            t2.join();
          if(t3.joinable())
            t3.join();
    }


  }

  std::cout << "...End of Test" << std::endl;
  return 0;
}
