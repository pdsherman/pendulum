/*
  File:   main.cpp
  Author: pdsherman
  Date:   April 2020

  Description: Create node for logging data to SQLite table
*/

#include <libs/logging/SqliteTable.hpp>

#include <pendulum/State.h>
#include <pendulum/StartLogging.h>

#include <ros/ros.h>
#include <boost/function.hpp>

#include <iostream>
#include <memory>
#include <string>

static const std::string db_filename = "EncoderData.db";
static std::shared_ptr<SqliteTable> table;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sqlite");
  ros::NodeHandle nh("sqlite");

  // Lambda used to be able to capture the Nodehandle object and be used in the callback function
  boost::function<bool(pendulum::StartLogging::Request &, pendulum::StartLogging::Response &)> logging_cb =
  [&nh](pendulum::StartLogging::Request &req, pendulum::StartLogging::Response &res)
  {
    if(!table) {
      table = std::make_shared<SqliteTable>(req.table_name);

      if(table->open_database(db_filename) && table->create_table()){
        table->set_start_time(req.start_time);
        table->subscribe(nh, req.topic_name);
        res.success = true;
      } else {
        res.success = false;
      }
    } else {
      ROS_WARN("Table already created");
    }

    return true;
  };

  ros::ServiceServer start_logging_server = nh.advertiseService("start_log", logging_cb);

  ros::Time s = ros::Time::now();

  std::cout << " sec: " << s.sec << std::endl;
  std::cout << "nsec: " << s.nsec << std::endl;

  std::cout << s.toSec() << std::endl;
  std::cout << s.toNSec() << std::endl;

  ros::spin();

  return 0;
}
