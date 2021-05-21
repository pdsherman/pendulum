/*
 * File:   main.cpp
 * Author: pdsherman
 * Date:   April 2020
 *
 * Description: Create node for logging data to SQLite table
*/

#include <libs/logging/SqliteTable.hpp>
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

static const std::string db_filename = "PendulumDatabase.db";
static DataHandler data_handler;

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
void msg_callback(const pendulum::LoggingData::ConstPtr &msg);

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
  ROS_INFO("SQLite Logging Node Started.");
  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

void msg_callback(const pendulum::LoggingData::ConstPtr &msg)
{
  data_handler.buffer_data(msg->test_name, msg->data);
}

bool logging_start_func(
  pendulum::LoggingStart::Request &req,
  pendulum::LoggingStart::Response &res,
  ros::NodeHandle &nh,
  ros::Subscriber &subscriber)
{
  
  if(!data_handler.logging_is_active()) {
    std::unique_ptr<SqliteTable> table = std::unique_ptr<SqliteTable>(new SqliteTable(req.table_name));
    if(table->open_database(db_filename) && table->create_table(req.header)){
      data_handler.set_table(std::move(table));
      if(data_handler.logging_begin()) {
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
  if(data_handler.logging_is_active()) {
    data_handler.logging_end();
    while(data_handler.logging_is_active()) { ros::Duration(0.2).sleep(); }
  }
  ROS_INFO("SQLite logging stopped.");
  return true;
}

bool drop_table(pendulum::LoggingDropTable::Request &req, pendulum::LoggingDropTable::Response &res)
{
  if(data_handler.logging_is_active()) {
    data_handler.logging_end();
    while(data_handler.logging_is_active()) { ros::Duration(0.2).sleep(); }
  }

  std::unique_ptr<SqliteTable> tbl = std::unique_ptr<SqliteTable>(new SqliteTable(req.table_name));
   if(tbl->open_database(db_filename) && tbl->delete_table()) {
    res.success = true;
    ROS_INFO("Log Table Dropped.");
  } else {
    res.success = false;
  }

  return true;
}

bool logging_buff_check(pendulum::LoggingBufferCheck::Request &req, pendulum::LoggingBufferCheck::Response &res)
{
  res.size     = data_handler.buffer_size();
  res.is_empty = data_handler.buffer_empty();
  return true;
}
