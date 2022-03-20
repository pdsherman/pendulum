
#include <libs/logging/SqliteTable.hpp>

#include <iostream>

SqliteTable::SqliteTable(std::shared_ptr<SqliteDatabase> database)
  : _db(std::move(database)), _insert_stmt(nullptr), _table_name()
{
}

bool SqliteTable::initialize_table(const std::string &table_name, const std::vector<std::string> &columns)
{
  _insert_stmt = nullptr;
  _table_name = table_name;

  if(!_table_name.empty()) {
    _insert_stmt = _db->create_insert_stmt(_table_name, columns);
    return _insert_stmt != nullptr;
  }

  return false;
}

bool SqliteTable::insert_row(const std::string &test_index, const std::vector<double> &data) {
  if(!_insert_stmt) { return false; }

  sqlite3_bind_text(_insert_stmt.get(),
                    sqlite3_bind_parameter_index(_insert_stmt.get(), ":x0"),
                    test_index.c_str(),
                    test_index.size(),
                    SQLITE_STATIC);

  for(size_t i = 0; i < data.size(); ++i) {
    std::string s = ":x" + std::to_string(i+1);
    sqlite3_bind_double(_insert_stmt.get(), sqlite3_bind_parameter_index(_insert_stmt.get(), s.c_str()), data[i]);
  }

  bool result = (sqlite3_step(_insert_stmt.get()) == SQLITE_DONE);
  sqlite3_clear_bindings(_insert_stmt.get());
  sqlite3_reset(_insert_stmt.get());
  return result;
}

bool SqliteTable::drop_table(void)
{
  return ready_to_insert() ? _db->delete_table(_table_name) : false;
}

bool SqliteTable::ready_to_insert(void) const
{
  return _insert_stmt != nullptr;
}

std::string SqliteTable::get_table_name(void) const
{
  return _table_name;
}
