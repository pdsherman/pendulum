/*
 * File:   main.cpp
 * Author: pdsherman
 * Date:   April 2020
 *
 * Description: Create node for logging data to SQLite table
*/

#include <libs/logging/SqliteTable.hpp>
#include <libs/logging/SqliteDatabase.hpp>
#include <libs/logging/DataHandler.hpp>

#include <pendulum/LoggingData.h>
#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingStop.h>
#include <pendulum/LoggingDropTable.h>
#include <pendulum/LoggingBufferCheck.h>

#include <ros/ros.h>
#include <boost/function.hpp>

#include <memory>
#include <string>
#include <utility>

static const std::string db_filename = "PendulumDatabase.db";
static std::shared_ptr<SqliteDatabase> db_connection = nullptr;
static std::map<std::string, std::pair<std::shared_ptr<DataHandler>, ros::Subscriber>> data_handlers;

// Function object for LoggingStart service.
// Creates a new SQLite table object and will subscribe to topic to insert into database
// Will need to use boost::bind to match required format for ROS service server
bool logging_start_func(
  pendulum::LoggingStart::Request &req,
  pendulum::LoggingStart::Response &res,
  ros::NodeHandle &nh);

// Function object for LoggingStop service.
// Creates a new SQLite table object and will subscribe to topic to insert into database
bool logging_stop_cb(pendulum::LoggingStop::Request &req, pendulum::LoggingStop::Response &res);

/// Function for LoggingDropTable service.
/// Stops logging if active, then drops table from SQLite database
bool drop_table(pendulum::LoggingDropTable::Request &req, pendulum::LoggingDropTable::Response &res);

/// Function for LoggingBufferCheck service.
/// Checks if logging has any buffered data to insert into database.
bool logging_buff_check(pendulum::LoggingBufferCheck::Request &req, pendulum::LoggingBufferCheck::Response &res);

/// Callback function for the ROS subscriber.
/// Saves msg for logging into SQLite table
/// @param [in] msg Published datapoint
/// @param [in] logger Database object to log msg to
void logging_buffer_msg(const pendulum::LoggingData::ConstPtr &msg, std::shared_ptr<DataHandler> logger);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sqlite");
  ros::NodeHandle nh("sqlite");

  /// Should only be a single instance of this object running at a time
  db_connection = std::make_shared<SqliteDatabase>();
  if(db_connection->open_database(db_filename)) {
    boost::function<bool(pendulum::LoggingStart::Request&, pendulum::LoggingStart::Response&)>
      logging_start_cb = boost::bind(logging_start_func, _1, _2, nh);

    ros::ServiceServer start_server = nh.advertiseService("start_log", logging_start_cb);
    ros::ServiceServer stop_server = nh.advertiseService("stop_log", logging_stop_cb);
    ros::ServiceServer buffer_check_server = nh.advertiseService("log_buffer_check", logging_buff_check);
    ros::ServiceServer remove_table_server = nh.advertiseService("drop_table", drop_table);

    ROS_INFO("SQLite Logging Node Started.");
    while(ros::ok()) {
      ros::spin();
    }
  } else {
    ROS_ERROR("SQLite logging node unable to open database.");
  }
  return 0;
}

void logging_buffer_msg(const pendulum::LoggingData::ConstPtr &msg, std::shared_ptr<DataHandler> logger)
{
  logger->buffer_data(msg->test_name, msg->data);
}

bool logging_start_func(
  pendulum::LoggingStart::Request &req,
  pendulum::LoggingStart::Response &res,
  ros::NodeHandle &nh)
{
  data_handlers[req.table_name] = std::pair<std::shared_ptr<DataHandler>, ros::Subscriber>();
  data_handlers[req.table_name].first = std::make_shared<DataHandler>(db_connection);
  if(data_handlers[req.table_name].first->create_table(req.table_name, req.header)){

    // Callback for messages
    boost::function<void(const pendulum::LoggingData::ConstPtr &)>
      msg_callback = boost::bind(logging_buffer_msg, _1, data_handlers[req.table_name].first);

    // Subscribe to topic
    std::string topic = req.topic_name;
    topic.insert(0, "/");
    data_handlers[req.table_name].second = nh.subscribe(topic, 2000, msg_callback);

    res.success = data_handlers[req.table_name].first->logging_begin();;
    ROS_INFO("SQLite logging started to table: %s", req.table_name.c_str());
    ROS_INFO("Subscribing to: %s", topic.c_str());
    return true;
  } else {
    data_handlers.erase(req.table_name);
    ROS_WARN("Failed to open DB or create table");
  }

  res.success = false;
  return false;
}

bool logging_stop_cb(pendulum::LoggingStop::Request &req, pendulum::LoggingStop::Response &res)
{
  if(data_handlers.find(req.table_name) == data_handlers.end()) {
    ROS_WARN("SQLite logging stop for invalid table name: %s", req.table_name.c_str());
    return false;
  }

  if(data_handlers[req.table_name].first->logging_is_active()) {
    data_handlers[req.table_name].first->logging_end();
    while(data_handlers[req.table_name].first->logging_is_active()) { ros::Duration(0.2).sleep(); }
  }

  ROS_INFO("SQLite logging stopped for table: %s", req.table_name.c_str());
  return true;
}

bool drop_table(pendulum::LoggingDropTable::Request &req, pendulum::LoggingDropTable::Response &res)
{
  res.success = false;

  if(data_handlers.find(req.table_name) != data_handlers.end() &&
      data_handlers[req.table_name].first->logging_is_active())
  {
    data_handlers[req.table_name].first->logging_end();
    while(data_handlers[req.table_name].first->logging_is_active()) { ros::Duration(0.2).sleep(); }
    data_handlers[req.table_name].first = nullptr;
  }

  if(db_connection->delete_table(req.table_name)) {
    res.success = true;
    ROS_INFO("SQLite Table %s Dropped.", req.table_name.c_str());
    return true;
  }

  ROS_INFO("SQLite failed to drop table: %s.", req.table_name.c_str());
  return false;
}

bool logging_buff_check(pendulum::LoggingBufferCheck::Request &req, pendulum::LoggingBufferCheck::Response &res)
{
  if(data_handlers.find(req.table_name) != data_handlers.end()) {
    res.size     = data_handlers[req.table_name].first->buffer_size();
    res.is_empty = data_handlers[req.table_name].first->buffer_empty();
    return true;
  }

  ROS_WARN("SQLite buffer check called with invalid table name: %s", req.table_name.c_str());
  res.size     = 0;
  res.is_empty = false;
  return false;
}
