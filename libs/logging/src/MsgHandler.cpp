/*
 * File:    MsgHandler.cpp
 * Author:  pdsherman
 * Date:    Sept. 2020
 */

#include <libs/logging/MsgHandler.hpp>

MsgHandler::MsgHandler(std::unique_ptr<SqliteTable> &&tbl) :
  _table(std::move(tbl)),
  _buffer(),
  _logging_thread(),
  _logging_active(false)
{
}

MsgHandler::~MsgHandler(void)
{
  _logging_active = false;
  _logging_thread.join();
}

void MsgHandler::set_table(std::unique_ptr<SqliteTable> &&tbl)
{
  if(!_logging_active.load()) {
    _table = std::move(tbl);
  }
}

void MsgHandler::buffer_data(MsgData &&data)
{
  std::lock_guard<std::mutex> lck(_buffer_mtx);
  _buffer.push(std::move(data));
}

bool MsgHandler::buffer_empty(void)
{
  std::lock_guard<std::mutex> lck(_buffer_mtx);
  return _buffer.empty();
}

size_t MsgHandler::buffer_size(void)
{
  std::lock_guard<std::mutex> lck(_buffer_mtx);
  return _buffer.size();
}

bool MsgHandler::logging_begin(void)
{
  if(!_logging_active.load())
  {
    _logging_thread = std::thread(&MsgHandler::logging_thread_func, this);
    return true;
  }
  return false;
}

void MsgHandler::logging_end(void)
{
  _logging_active = false;
  _logging_thread.join();
}

bool MsgHandler::logging_is_active(void)
{
  return _logging_active.load();
}

void MsgHandler::logging_thread_func(void)
{
  _logging_active = true;
  bool insert = false;
  MsgData d;
  while(_logging_active.load())
  {
    {
      std::lock_guard<std::mutex> lck(_buffer_mtx);
      if(!_buffer.empty())
      {
        d = _buffer.front();
        _buffer.pop();
        insert = true;
      }
    }

    if(insert) {
      _table->insert_row(d.timestamp, d.test_time, d.x, d.theta);
      insert = false;
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}
