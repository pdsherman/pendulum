
#include <libs/util/log_util.hpp>
#include <libs/logging/SqliteDatabase.hpp>
#include <libs/logging/SqliteTable.hpp>

#include <sqlite3.h>
#include <iostream>
#include <memory>
#include <unistd.h>

namespace util {


static const std::string database =
    "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/PendulumDatabase.db";

/// Attempt to open a connection to a SQLite database in READ-ONLY mode
/// @param [in] db_filename Name (full path) to database
/// @return Pointer to database connection (NULL if open fails)
static std::shared_ptr<sqlite3> open_database(const std::string &db_filename)
{
  static constexpr int flags = SQLITE_OPEN_READONLY;

  sqlite3 *db_local = nullptr;
  int err = sqlite3_open_v2(db_filename.c_str(), &db_local, flags, nullptr);

  if(err == SQLITE_OK){
    // The following PRAGMA's improve insertion times by ~99%
    // Sacrifices robustness and ability for multiple database connections
    // Okay for a personal project like this where its not a big deal
    sqlite3_exec(db_local, "PRAGMA synchronous = OFF", NULL, NULL, NULL);
    sqlite3_exec(db_local, "PRAGMA journal_mode = OFF", NULL, NULL, NULL);
    sqlite3_exec(db_local, "PRAGMA locking_mode = EXCLUSIVE", NULL, NULL, NULL);

    std::shared_ptr<sqlite3> db = std::shared_ptr<sqlite3>(db_local, [](sqlite3 *p) { sqlite3_close(p); });
    return db;
  }
  return nullptr;
}

std::map<std::string, std::vector<double>> read_data_from_db(const std::string &select_stmt)
{
  std::map<std::string, std::vector<double>> data;

  std::shared_ptr<sqlite3> db = open_database(database);
  if(db) {
    sqlite3_stmt* stmt;
    if(sqlite3_prepare_v2(db.get(), select_stmt.c_str(), -1, &stmt, nullptr) == SQLITE_OK) {

      bool first = true;
      int num_col = 0;
      while( sqlite3_step(stmt) == SQLITE_ROW) {
        // First time in loop, initialize map with column names and empty vector
        if(first) {
          num_col = sqlite3_column_count(stmt);
          for(int i = 1; i < num_col; ++i) {
            std::string col_name = std::string(sqlite3_column_name(stmt, i));
            data[col_name] = std::vector<double>();
          }
          first = false;
        }

        for(int i = 1; i < num_col; ++i) {
          std::string col_name = std::string(sqlite3_column_name(stmt, i));
          double x = sqlite3_column_double(stmt, i);
          data[col_name].push_back(x);
        }
      }
    }

    // Always finalize, even if failed to prepare
    sqlite3_finalize(stmt);
  }


  return data;
}

bool write_data_to_db(const std::string &table_name,
        const std::string &test_name,
        const std::vector<std::string> &header,
        const std::vector<std::vector<double>> &data)
{
  std::shared_ptr<SqliteDatabase> db = std::make_shared<SqliteDatabase>();
  if(db->open_database("PendulumDatabase.db")) {
    usleep(150000);
    SqliteTable tbl(db);
    if(tbl.initialize_table(table_name, header)) {
      usleep(150000);
      for(const auto &row : data) {
        if(!tbl.insert_row(test_name, row)) {
          return false;
        }
      }
      return true;
    }
  }

  return false;
}

} // namespace util
