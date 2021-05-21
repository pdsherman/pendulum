
#include <libs/logging/SqliteTable.hpp>

#include <iostream>

const std::string SqliteTable::kDatabaseDirectory =
            "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/";

SqliteTable::SqliteTable(const std::string &table_name)
  : _db(nullptr), _insert_stmt(nullptr), _table_exists(false), _table_name(table_name)
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
    // PRAGMA's Improve insertion times by ~99% by sacrficing robustness.
    // Okay for personal project like this where its not a big deal
    // if database is corrupted and is lost.
    sqlite3_exec(db_local, "PRAGMA synchronous = OFF", NULL, NULL, NULL);
    sqlite3_exec(db_local, "PRAGMA journal_mode = MEMORY", NULL, NULL, NULL);

    _db = std::shared_ptr<sqlite3>(db_local, [](sqlite3 *p) { sqlite3_close(p); } );
    return true;
  }
  return false; //"Failed to create table in SQLite database"
}

bool SqliteTable::create_table(const std::vector<std::string> &columns) {
  if(!_db || _table_name.empty() || columns.empty()) { return false; }

  _table_exists = false;

  std::string cmd = "CREATE TABLE IF NOT EXISTS " + _table_name + " (";
  cmd += columns[0] + " STRING, ";
  for(size_t i = 1; i < columns.size(); ++i) {
    cmd += columns[i] + " REAL";
    if (i != columns.size()-1) { cmd += ", "; }
  }
  cmd += ")";

  sqlite3_stmt *st = nullptr;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &st, nullptr) != SQLITE_OK) {
    sqlite3_finalize(st);
    return false; //"Failed to create table in SQLite database"
  }

  _table_exists = (sqlite3_step(st) == SQLITE_DONE);
  sqlite3_finalize(st);

  if(_table_exists)
    return creat_insert_stmt(columns);
  return false;
}

bool SqliteTable::creat_insert_stmt(const std::vector<std::string> &columns)
{
  std::string cmd = "INSERT INTO " + _table_name + " (";
  for(auto &s : columns) { cmd += s + ", "; }
  cmd.pop_back();
  cmd.pop_back();
  cmd += ") VALUES (";
  for(size_t i = 0; i < columns.size(); ++i) {
    cmd += ":x" + std::to_string(i);
    if (i != columns.size()-1) { cmd += ", "; }
  }
  cmd += ")";

  sqlite3_stmt* stmt;
  if(sqlite3_prepare_v2(_db.get(), cmd.c_str(), -1, &stmt, nullptr) != SQLITE_OK) {
    sqlite3_finalize(stmt);
    _insert_stmt = nullptr;
    return false;
  }

  _insert_stmt = std::shared_ptr<sqlite3_stmt>(stmt,[](sqlite3_stmt *p) { sqlite3_finalize(p); });
  return true;
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

  _table_exists = false;
  _insert_stmt = nullptr;

  sqlite3_step(st);
  sqlite3_finalize(st);
  return true;
}

bool SqliteTable::insert_row(const std::string &test_index, const std::vector<double> &data) {
  if(!_db || !_table_exists || !_insert_stmt) { return false; }

  sqlite3_bind_text(_insert_stmt.get(), sqlite3_bind_parameter_index(_insert_stmt.get(), ":x0"),
    test_index.c_str(), test_index.size(), SQLITE_STATIC);
  for(size_t i = 0; i < data.size(); ++i) {
    std::string s = ":x" + std::to_string(i+1);
    sqlite3_bind_double(_insert_stmt.get(), sqlite3_bind_parameter_index(_insert_stmt.get(), s.c_str()), data[i]);
  }

  bool result = (sqlite3_step(_insert_stmt.get()) == SQLITE_DONE);
  sqlite3_clear_bindings(_insert_stmt.get());
  sqlite3_reset(_insert_stmt.get());
  return result;
}
