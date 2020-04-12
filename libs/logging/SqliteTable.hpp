/*
 * @file:   SqliteTable.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Class for subscibing to ROS topic and logging with SQLite table.
 */

#pragma once

#include <pendulum/State.h>

#include <sqlite3.h>
#include <ros/ros.h>

#include <string>
#include <memory>


class SqliteTable {
public:
  /// Constructor
  /// @param [in] database_file Filename of SQLite database
  SqliteTable(const std::string &table_name);

  /// Destructor
  ~SqliteTable(void);

  /// Open database.
  /// @param [in] database_file Filename of database
  /// @note Empty string will create a temporary database that is destroyed
  /// at object destruction
  /// @return True if database created and/or opened successfully
  bool open_database(const std::string &database_file = "");

  /// Creates the table in that database if it doesn't already exist
  /// @return True if table created successfully
  bool create_table(void);

  /// Subscribe table to topic to begin logging to database
  /// @param [in] nh ROS node handle object
  /// @param [in] topic Name of the ROS topic to subscribe to
  void subscribe(ros::NodeHandle &nh, const std::string &topic);

private:
  /// Callback function for the ROS subscriber. Inserts
  /// row into table.
  /// @param [in] msg Published datapoint
  void callback(const pendulum::State::ConstPtr &msg);

  /// Insert row into table
  /// @param [in] timestamp Timestamp when datapoint sampled
  /// @param [in] test_time Time from the start of the test
  /// @param [in] x State variable X
  /// @param [in] theta State variable theta
  /// @return True if row inserted into table successfully
  bool insert_row(const double timestamp, const double test_time, const double x, const double theta);

  /// Pointer to database
  std::shared_ptr<sqlite3> _db;

  /// ROS subscriber object
  ros::Subscriber _subscriber;

  /// Name of table in database
  std::string _table_name;

  /// Does the table exisit in the database
  bool _table_exists;

  /// Flags for creating table
  static constexpr int kDbFlags = SQLITE_OPEN_READWRITE | SQLITE_OPEN_CREATE;

  /// Directory for database
  static constexpr const char *const kDatabaseDirectory =
              "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/libs/logging/data/";
};
