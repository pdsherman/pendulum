
#include <libs/logging/SqliteTable.hpp>

#include <iostream>

const std::string SqliteTable::kDatabaseDirectory =
            "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/";

SqliteTable::SqliteTable(const std::string &table_name)
  : _db(nullptr), _table_name(table_name)
{
}

SqliteTable::~SqliteTable(void)
{
  _db.reset();
  sqlite3_shutdown();
}

std::string SqliteTable::get_table_name(void) const
{
  return _table_name;
}

bool SqliteTable::open_database(const std::string &database_file)
{
  static constexpr int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;

  sqlite3 *db_local = nullptr;
  std::string full_path = kDatabaseDirectory + database_file;
  int r = sqlite3_open_v2(full_path.c_str(), &db_local, flags, nullptr);

  if(r == SQLITE_OK){
    _db = std::shared_ptr<sqlite3>(db_local, [](sqlite3 *p) { sqlite3_close(p); } );
    return true; //"Failed to create table in SQLite database"
  }
  return false;
}

bool SqliteTable::create_table(void) {
  if(!_db || _table_name.empty()) { return false; }

  _table_exists = false;

  std::string cmd = "CREATE TABLE IF NOT EXISTS " + _table_name;
  cmd += " (timestamp REAL, test_time_s REAL, x REAL, theta REAL)";

  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, nullptr) != SQLITE_OK) {
    sqlite3_finalize(st);
    return false; //"Failed to create table in SQLite database"
  }

  _table_exists = (sqlite3_step(st) == SQLITE_DONE);
  sqlite3_finalize(st);
  return _table_exists;
}

bool SqliteTable::delete_table(void)
{
  if(!_db || _table_name.empty()) { return false; }

  std::string cmd = "DROP TABLE IF EXISTS " + _table_name;
  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, nullptr) != SQLITE_OK) {
    sqlite3_finalize(st);
    return false; // "Failed to drop table in SQLite database"
  }

  //ROS_INFO("Dropped table %s from SQLite database", _table_name.c_str());
  _table_exists = false;
  sqlite3_step(st);
  sqlite3_finalize(st);
  return true;
}

bool SqliteTable::insert_row(const double timestamp, const double test_time, const double x, const double theta) {
  if(!_db || !_table_exists) { return false; }

  std::string cmd = "INSERT INTO " + _table_name;
  cmd += " (timestamp, test_time_s, x, theta) VALUES (:ts, :tt, :x, :theta)";

  sqlite3_stmt* stmt;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
    sqlite3_finalize(stmt);
    return false;
  }

  sqlite3_bind_double(stmt, sqlite3_bind_parameter_index(stmt, ":ts"), timestamp);
  sqlite3_bind_double(stmt, sqlite3_bind_parameter_index(stmt, ":tt"), test_time);
  sqlite3_bind_double(stmt, sqlite3_bind_parameter_index(stmt, ":x"), x);
  sqlite3_bind_double(stmt, sqlite3_bind_parameter_index(stmt, ":theta"), theta);

  bool result = sqlite3_step(stmt) == SQLITE_DONE;
  sqlite3_finalize(stmt);
  return result;
}
