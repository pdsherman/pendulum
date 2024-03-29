/*
 * File:    DataHandlerhpp
 * Author:  pdsherman
 * Date:    Sept. 2020
 *
 * Description: Object to handle multi-threading of
 *              buffering and inserting State data into database.
 */

#pragma once

#include <libs/logging/SqliteDatabase.hpp>
#include <libs/logging/SqliteTable.hpp>
#include <libs/threadsafe_queue/ThreadsafeQueue.hpp>

#include <memory>
#include <thread>
#include <atomic>

class DataHandler
{
public:

  /// Constructor
  /// @param [in] db SQLite database connection object
  DataHandler(std::shared_ptr<SqliteDatabase> db);

  /// Delete copy constructor
  DataHandler(const DataHandler&) = delete;

  /// Delete Assignment operator
  DataHandler operator=(const DataHandler&) = delete;

  /// Destructor
  ~DataHandler(void);

  /// Set the SqliteTable to use for logging
  /// @param [in] tbl Sqlite table to use
  bool create_table(const std::string &tbl, const std::vector<std::string> &header);

  /// Drop table if it exists
  /// @return True if table exists and was able to be dropped form database
  bool drop_table(void);

  /// Insert data into buffer to be inserted into SQLite table
  /// @param [in] data New datapoint to buffer
  void buffer_data(const std::string &test_name, const std::vector<double> &data);

  /// Check if the data buffer has any remaining data points to insert.
  /// @return True if buffer is empty.
  bool buffer_empty(void);

  /// Get the current size of buffer
  size_t buffer_size(void);

  /// Start-up a thread to begin looping to removing data from
  /// buffer and inserting into the SQLite table
  bool logging_begin(void);

  /// Ends the logging thread
  void logging_end(void);

  /// Check if logging is currently active
  /// @return True if active flag is true
  bool logging_is_active(void);

private:

  using Row = std::pair<std::string, std::vector<double>>;

  /// Function to run in thread.
  /// Will constantly loop and attempt to
  /// remove data from buffer and insert into database
  void logging_thread_func(void);

  /// Interface object for writing to SQLite database
  std::unique_ptr<SqliteTable> _table;

  /// Buffer object for data to be logged
  ThreadsafeQueue<Row> _buffer;

  /// Thread object to run logging function
  std::thread _logging_thread;

  /// Flag to signal thread to stop running
  std::atomic<bool> _logging_active;
};
