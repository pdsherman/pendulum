
#include <libs/logging/SqliteDatabase.hpp>

#include <iostream>

const std::string SqliteDatabase::kDatabaseDirectory =
  "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/";

SqliteDatabase::SqliteDatabase(void) :
  _db(nullptr)
{
}

SqliteDatabase::~SqliteDatabase(void)
{
  _db.reset();
  sqlite3_shutdown();
}

bool SqliteDatabase::open_database(const std::string &database_file)
{
  static constexpr int flags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;

  sqlite3 *db_local = nullptr;
  std::string full_path = database_file.empty() ? "" : kDatabaseDirectory + database_file;

  int err = sqlite3_open_v2(full_path.c_str(), &db_local, flags, NULL);
  if(err == SQLITE_OK){
    // The following PRAGMA's improve insertion times by ~99%
    // Sacrifices robustness and ability for multiple database connections
    // Okay for a personal project like this where its not a big deal
    sqlite3_exec(db_local, "PRAGMA synchronous = OFF", NULL, NULL, NULL);
    sqlite3_exec(db_local, "PRAGMA journal_mode = OFF", NULL, NULL, NULL);
    sqlite3_exec(db_local, "PRAGMA locking_mode = EXCLUSIVE", NULL, NULL, NULL);

    _db = std::shared_ptr<sqlite3>(db_local, [](sqlite3 *p) { sqlite3_close(p); } );
    return true;
  }
  return false;
}

std::shared_ptr<sqlite3_stmt> SqliteDatabase::create_insert_stmt(
  const std::string &table_name,
  const std::vector<std::string> &column_names)
{
  // Make sure table exists
  if (!create_table(table_name, column_names)) { return nullptr; }

  // Create SQLite insert statement
  // INSERT INTO <table-name> (<name-1>, <name-2>, ... <name-n>) VALUES ( :x1, :x2, ... :xn)
  std::string cmd = "INSERT INTO " + table_name + " (";
  for(auto &s : column_names) { cmd += s + ", "; }
  cmd.pop_back();
  cmd.pop_back();
  cmd += ") VALUES (";
  for(size_t i = 0; i < column_names.size(); ++i) {
    cmd += ":x" + std::to_string(i);
    if (i != column_names.size()-1) { cmd += ", "; }
  }
  cmd += ")";

  sqlite3_stmt* stmt;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
    sqlite3_finalize(stmt);
    return nullptr;
  }

  return std::shared_ptr<sqlite3_stmt>(stmt, [](sqlite3_stmt *p) { sqlite3_finalize(p); });
}

bool SqliteDatabase::create_table(const std::string &table_name, const std::vector<std::string> &columns) {
  if(!_db || columns.empty()) { return false; }

  std::string cmd = "CREATE TABLE IF NOT EXISTS " + table_name + " (";
  cmd += columns[0] + " STRING, ";
  for(size_t i = 1; i < columns.size(); ++i) {
    cmd += columns[i] + " REAL";
    if (i != columns.size()-1) { cmd += ", "; }
  }
  cmd += ")";

  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, nullptr) != SQLITE_OK) {
    sqlite3_finalize(st);
    return false; //
  }

  bool result = (sqlite3_step(st) == SQLITE_DONE);
  sqlite3_finalize(st);
  return result;
}

bool SqliteDatabase::delete_table(const std::string &table_name)
{
  if(!_db || table_name.empty()) { return false; }

  std::string cmd = "DROP TABLE IF EXISTS " + table_name;
  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, NULL) != SQLITE_OK) {
    sqlite3_finalize(st);
    return false; // Failed to drop table in SQLite database
  }

  sqlite3_step(st);
  sqlite3_finalize(st);
  return true;
}
