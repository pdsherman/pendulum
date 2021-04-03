/*
 * File:    DataHandler.cpp
 * Author:  pdsherman
 * Date:    Sept. 2020
 */

#include <libs/logging/DataHandler.hpp>

DataHandler::DataHandler(std::unique_ptr<SqliteTable> &&tbl) :
  _table(std::move(tbl)),
  _buffer(),
  _logging_thread(),
  _logging_active(false)
{
}

DataHandler::~DataHandler(void)
{
  logging_end();
}

void DataHandler::set_table(std::unique_ptr<SqliteTable> &&tbl)
{
  if(!_logging_active.load()) {
    _table = std::move(tbl);
  }
}

void DataHandler::buffer_data(const std::string &test_name, const std::vector<double> &data)
{
  std::lock_guard<std::mutex> lck(_buffer_mtx);
  _buffer.push({test_name, data});
}

bool DataHandler::buffer_empty(void)
{
  std::lock_guard<std::mutex> lck(_buffer_mtx);
  return _buffer.empty();
}

size_t DataHandler::buffer_size(void)
{
  std::lock_guard<std::mutex> lck(_buffer_mtx);
  return _buffer.size();
}

bool DataHandler::logging_begin(void)
{
  if(!_logging_active.load())
  {
    _logging_thread = std::thread(&DataHandler::logging_thread_func, this);
    return true;
  }
  return false;
}

void DataHandler::logging_end(void)
{
  _logging_active = false;
  if(_logging_thread.joinable())
    _logging_thread.join();
}

bool DataHandler::logging_is_active(void)
{
  return _logging_active.load();
}

void DataHandler::logging_thread_func(void)
{
  _logging_active = true;
  bool insert = false;
  Row row;
  while(_logging_active.load()) {
    {
      std::lock_guard<std::mutex> lck(_buffer_mtx);
      if(!_buffer.empty()) {
        row = _buffer.front();
        _buffer.pop();
        insert = true;
      }
    }

    if(insert) {
      _table->insert_row(row.first, row.second);
      insert = false;
      std::this_thread::sleep_for(std::chrono::microseconds(1));
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
  }
}
