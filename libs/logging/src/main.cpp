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
#include <pendulum/LoggingDropTable.h>
#include <pendulum/LoggingBufferEmpty.h>

#include <ros/ros.h>
#include <boost/function.hpp>

#include <memory>
#include <string>

static const std::string db_filename = "EncoderData.db";
static MsgHandler msg_handler;

/// Function for LoggingDropTable service.
// Stops logging if active, then drops table from SQLite database
bool drop_table(pendulum::LoggingDropTable::Request &req, pendulum::LoggingDropTable::Response &res);

/// Function for LoggingBufferEmpty service.
/// Checks if logging has any buffered data to insert into database.
bool logging_buff_empty(pendulum::LoggingBufferEmpty::Request &req, pendulum::LoggingBufferEmpty::Response &res);

/// Callback function for the ROS subscriber. Saves
/// msg for logging into SQLite table
/// @param [in] msg Published datapoint
void msg_callback(const pendulum::State::ConstPtr &msg);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sqlite");
  ros::NodeHandle nh("sqlite");
  ros::Subscriber subscriber;

  // Function object for LoggingStart service.
  // Creates a new SQLite table object and will subscribing to topic to insert into database
  // Lambda used to capture the ROS NodeHandle and Subscriber objects for use in function
  // Using boost since ros does not accept std::function. (as of Aug. 2020)
  boost::function<bool(pendulum::LoggingStart::Request &, pendulum::LoggingStart::Response &)> start_logging_cb =
  [&nh, &subscriber](pendulum::LoggingStart::Request &req, pendulum::LoggingStart::Response &res)
  {
    if(!msg_handler.logging_is_active()) {
      std::unique_ptr<SqliteTable> table = std::unique_ptr<SqliteTable>(new SqliteTable(req.table_name));
      if(table->open_database(db_filename) && table->create_table()){
        msg_handler.set_table(std::move(table));
        if(msg_handler.logging_begin()) {
          // Subscribe to topic
          std::string topic = req.topic_name;
          topic.insert(0, "/");
          subscriber = nh.subscribe(topic, 500, &msg_callback);

          res.success = true;
          ROS_INFO("SQLite logging started");
        }
      } else {
        table = nullptr;
        res.success = false;
        ROS_WARN("Failed to open DB or create table");
      }
    } else {
      ROS_WARN("Table already created");
    }

    return true;
  };

  ros::ServiceServer logging_stat_server = nh.advertiseService("start_log", start_logging_cb);
  ros::ServiceServer logging_buffer_empty_server = nh.advertiseService("log_buffer_empty", logging_buff_empty);
  ros::ServiceServer remove_table_server = nh.advertiseService("drop_table", drop_table);

  ros::Rate rate(1/0.005);
  while(ros::ok())
  {
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

bool drop_table(pendulum::LoggingDropTable::Request &req, pendulum::LoggingDropTable::Response &res)
{
  if(msg_handler.logging_is_active()) {
    msg_handler.logging_end();
    while(msg_handler.logging_is_active()) { usleep(200000); }
  }

  std::unique_ptr<SqliteTable> tbl = std::unique_ptr<SqliteTable>(new SqliteTable(req.table_name));
  if(tbl->open_database(db_filename) && tbl->delete_table())
    res.success = true;
  else
    res.success = false;


  return true;
}

bool logging_buff_empty(pendulum::LoggingBufferEmpty::Request &req, pendulum::LoggingBufferEmpty::Response &res)
{
  res.is_empty = msg_handler.buffer_empty();
  return true;
}
