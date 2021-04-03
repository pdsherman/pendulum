/*
 * @file:   SqliteTable.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Class for subscibing to ROS topic and logging with SQLite table.
 */

#pragma once

#include <libs/function_timer/FunctionTimer.hpp>

#include <sqlite3.h>

#include <string>
#include <memory>

class SqliteTable {
public:
  enum class DataTypes {
    kNull,
    kInteger,
    kReal,
    kText,
    kBlob
  };

  /// Constructor
  /// @param [in] database_file Filename of SQLite database
  SqliteTable(const std::string &table_name);

  /// Destructor
  ~SqliteTable(void);

  /// @return Get the the table name
  std::string get_table_name(void) const;

  /// Open database.
  /// @param [in] database_file Filename of database
  /// @note Empty string will create a temporary database that is destroyed
  /// at object destruction
  /// @return True if database created and/or opened successfully
  bool open_database(const std::string &database_file = "");

  /// Creates the table in that database if it doesn't already exist
  /// @param [in] columns Vector of strings naming the table columns
  /// @return True if table created successfully
  bool create_table(const std::vector<std::string> &columns);

  /// Remove table from the database if it exists
  /// @return True if table existed in database and was removed
  bool delete_table(void);

  /// Insert row into table
  /// @param [in] timestamp Timestamp when datapoint sampled
  /// @param [in] test_time Time from the start of the test
  /// @return True if row inserted into table successfully
  bool insert_row(const std::string &test_index, const std::vector<double> &data);

private:

  bool creat_insert_stmt(const std::vector<std::string> &colums);

  /// Pointer to database
  std::shared_ptr<sqlite3> _db;

  /// Insertion statement to persists for life of logging process
  std::shared_ptr<sqlite3_stmt> _insert_stmt;

  /// Does the table exisit in the database
  bool _table_exists;

  /// Name of table in database
  std::string _table_name;

  /// Flags for creating table
  static constexpr int kDbFlags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;

  /// Directory for database
  static const std::string kDatabaseDirectory;
};
