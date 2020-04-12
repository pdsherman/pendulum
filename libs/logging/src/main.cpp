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
static std::vector<std::unique_ptr<SqliteTable>> tables;

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "sqlite");
  ros::NodeHandle nh("sqlite");

  // Lambda used to be able to capture the Nodehandle object and be used in the callback function
  boost::function<bool(pendulum::StartLogging::Request &, pendulum::StartLogging::Response &)> logging_cb =
  [&nh](pendulum::StartLogging::Request &req, pendulum::StartLogging::Response &res) {
    std::unique_ptr<SqliteTable> table = std::unique_ptr<SqliteTable>(new SqliteTable(req.table_name));

    if(table->open_database(db_filename) && table->create_table()){
      table->subscribe(nh, req.topic_name);
      tables.push_back(std::move(table));
      res.success = true;
    } else {
      res.success = false;
    }
    return true;
  };

  ros::ServiceServer start_logging_server = nh.advertiseService("start_log", logging_cb);

  ros::Rate rate(500);
  while(ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
