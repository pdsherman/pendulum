/*
 * File:    DataHandlerhpp
 * Author:  pdsherman
 * Date:    Sept. 2020
 *
 * Description: Object to handle multi-threading of
 *              buffering and inserting State data into database.
 */

#pragma once

#include <libs/logging/SqliteTable.hpp>

#include <memory>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

class DataHandler
{
public:

  /// Default constructor
  DataHandler(void) = default;

  /// Constructor
  /// @param [in] tbl SQLite table for inserting data
  DataHandler(std::unique_ptr<SqliteTable> &&tbl);

  /// Destructor
  ~DataHandler(void);

  /// Set the SqliteTable to use for logging
  /// @param [in] tbl Sqlite table to use
  void set_table(std::unique_ptr<SqliteTable> &&tbl);

  /// Insert data into buffer to be inserted into SQLite table
  /// @param [in] data New datapoint to buffer
  void buffer_data(const std::string &test_name, const std::vector<double> &data);

  /// Check if the data buffer has any remaining data points to insert.
  /// @return True if buffer is empty.
  bool buffer_empty(void);

  /// Get the current size of buffer
  size_t buffer_size(void);

  /// Start-up a thread to begin removing data from
  /// buffer and inserting into the SQLite table
  bool logging_begin(void);

  void logging_end(void);

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
  std::queue<Row> _buffer;

  /// Mutex for lokcing acces to buffer
  std::mutex _buffer_mtx;

  /// Thread object to run logging function
  std::thread _logging_thread;

  /// Flag to signal thread to stop running
  std::atomic<bool> _logging_active;
};
