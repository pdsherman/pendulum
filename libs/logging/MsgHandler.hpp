/*
 * File:    MsgHandlerhpp
 * Author:  pdsherman
 * Date:    Sept. 2020
 *
 * Description: Object to handle multi-threading of
 *              buffering and inserting State data into database.
 */

#include <libs/logging/SqliteTable.hpp>

#include <memory>
#include <queue>
#include <mutex>
#include <thread>
#include <atomic>

class MsgHandler
{
public:
  struct MsgData {
    double timestamp;
    double theta;
    double x;
    double test_time;
  };

  /// Default constructor
  MsgHandler(void) = default;

  /// Constructor
  /// @param [in] tbl SQLite table for inserting data
  MsgHandler(std::unique_ptr<SqliteTable> &&tbl);

  /// Destructor
  ~MsgHandler(void);

  /// Set the SqliteTable to use for logging
  /// @param [in] tbl Sqlite table to use
  void set_table(std::unique_ptr<SqliteTable> &&tbl);

  /// Insert data into buffer to be inserted into SQLite table
  /// @param [in] data New datapoint to buffer
  void buffer_data(MsgData &&data);

  /// Check if the data buffer has any remaining data point
  /// to insert.
  /// @return True if buffer is empty.
  bool buffer_empty(void);

  /// Start-up a thread to begin removing data from
  /// buffer and inserting into the SQLite table
  bool logging_begin(void);

  void logging_end(void);

  bool logging_is_active(void);
private:

  void logging_thread_func(void);

  std::unique_ptr<SqliteTable> _table;

  std::queue<MsgData> _buffer;

  std::mutex _buffer_mtx;

  std::thread _logging_thread;

  std::atomic<bool> _logging_active;
};
