/*
 * File:   main.cpp
 * Author: pdsherman
 * Date:   April 2020
 *
 * Description: Create node for logging data to SQLite table
*/

#include <libs/logging/SqliteTable.hpp>
#include <libs/logging/MsgHandler.hpp>

#include <pendulum/State.h>
#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingStop.h>
#include <pendulum/LoggingDropTable.h>
#include <pendulum/LoggingBufferCheck.h>

#include <ros/ros.h>
#include <boost/function.hpp>

#include <memory>
#include <string>

static const std::string db_filename = "PendulumDatabase.db";
static MsgHandler msg_handler;

// Function object for LoggingStart service.
// Creates a new SQLite table object and will subscribe to topic to insert into database
// Will need to use boost::bind to match required format for ROS service server
bool logging_start_func(
  pendulum::LoggingStart::Request &req,
  pendulum::LoggingStart::Response &res,
  ros::NodeHandle &nh,
  ros::Subscriber &subscriber);

// Function object for LoggingStop service.
// Creates a new SQLite table object and will subscribe to topic to insert into database
// Lambda used to capture the ROS NodeHandle and Subscriber objects for use in function
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
void msg_callback(const pendulum::State::ConstPtr &msg);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sqlite");
  ros::NodeHandle nh("sqlite");
  ros::Subscriber subscriber;

  // Using boost since ros does not accept std::function. (as of Aug. 2020)
  boost::function<bool(pendulum::LoggingStart::Request &, pendulum::LoggingStart::Response &)>
    logging_start_cb = boost::bind(logging_start_func, _1, _2, nh, subscriber);

  ros::ServiceServer start_server = nh.advertiseService("start_log", logging_start_cb);
  ros::ServiceServer stop_server = nh.advertiseService("stop_log", logging_stop_cb);
  ros::ServiceServer buffer_check_server = nh.advertiseService("log_buffer_check", logging_buff_check);
  ros::ServiceServer remove_table_server = nh.advertiseService("drop_table", drop_table);

  ros::Rate rate(1/0.01);
  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

void msg_callback(const pendulum::State::ConstPtr &msg)
{
  MsgHandler::MsgData d;
  d.timestamp = msg->header.stamp.toSec();
  d.theta     = msg->theta;
  d.x         = msg->x;
  d.test_time = msg->test_time_s;

  msg_handler.buffer_data(std::move(d));
}

bool logging_start_func(
  pendulum::LoggingStart::Request &req,
  pendulum::LoggingStart::Response &res,
  ros::NodeHandle &nh,
  ros::Subscriber &subscriber)
{
  if(!msg_handler.logging_is_active()) {
    std::unique_ptr<SqliteTable> table = std::unique_ptr<SqliteTable>(new SqliteTable(req.table_name));
    if(table->open_database(db_filename) && table->create_table()){
      msg_handler.set_table(std::move(table));
      if(msg_handler.logging_begin()) {
        // Subscribe to topic
        std::string topic = req.topic_name;
        topic.insert(0, "/");
        subscriber = nh.subscribe(topic, 2000, &msg_callback);

        res.success = true;
        ROS_INFO("SQLite logging started");
      }
    } else {
      table = nullptr;
      res.success = false;
      ROS_WARN("Failed to open DB or create table");
    }
  } else {
    res.success = false;
    ROS_WARN("Logging already active");
  }

  return true;
}

bool logging_stop_cb(pendulum::LoggingStop::Request &req, pendulum::LoggingStop::Response &res)
{
  if(msg_handler.logging_is_active()) {
    msg_handler.logging_end();
    while(msg_handler.logging_is_active()) { ros::Duration(0.2).sleep(); }
  }
  ROS_INFO("Logging Stopped");
  return true;
}

bool drop_table(pendulum::LoggingDropTable::Request &req, pendulum::LoggingDropTable::Response &res)
{
  if(msg_handler.logging_is_active()) {
    msg_handler.logging_end();
    while(msg_handler.logging_is_active()) { ros::Duration(0.2).sleep(); }
  }

  std::unique_ptr<SqliteTable> tbl = std::unique_ptr<SqliteTable>(new SqliteTable(req.table_name));
  if(tbl->open_database(db_filename) && tbl->delete_table())
    res.success = true;
  else
    res.success = false;

  return true;
}

bool logging_buff_check(pendulum::LoggingBufferCheck::Request &req, pendulum::LoggingBufferCheck::Response &res)
{
  res.size     = msg_handler.buffer_size();
  res.is_empty = msg_handler.buffer_empty();
  return true;
}
