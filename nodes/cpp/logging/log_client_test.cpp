/*
  File:   log_clien_test.cpp
  Author: pdsherman
  Date:   Apr. 2021

  Description: Client for testing SQL node
*/

#include <pendulum/LoggingData.h>
#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingDropTable.h>
#include <pendulum/LoggingBufferCheck.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <string>
#include <thread>
#include <random>
#include <vector>
#include <cmath>

/// Connect to logging node and initialize table for database
bool start_logging(ros::NodeHandle &nh,
  const std::string &name,
  const std::string &log_table_name,
  const std::vector<std::string> &header);

/// Run test to send trig functions to logging table
void log_trig_functions(ros::NodeHandle &nh);

/// Run test to normal distribution data to logging table
void log_normal_distribution(ros::NodeHandle &nh);

/// Intentionally make invalid calls to make sure errors appear on server side
void test_errors(ros::NodeHandle &nh);

int main(int argc, char *argv[])
{
  if(std::string(argv[1]) == std::string("trig")) {
    ros::init(argc, argv, "log_client_test_trig");
    ros::NodeHandle nh;
    log_trig_functions(nh);
      ROS_INFO("Ending SQL client test.");
  } else {
    ros::init(argc, argv, "log_client_test_normal");
    ros::NodeHandle nh;
    log_normal_distribution(nh);
    ROS_INFO("Ending SQL client test.");
  }

  return 0;
}

bool start_logging(ros::NodeHandle &nh,
  const std::string &name,
  const std::string &log_table_name,
  const std::vector<std::string> &header)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingDropTable>("/sqlite/drop_table");
  if(sql_client.exists()) {
    pendulum::LoggingDropTable drop_table;
    drop_table.request.table_name = log_table_name;

    if(!sql_client.call(drop_table) || !drop_table.response.success)
      ROS_WARN("Failure to drop table");
  }

  sql_client = nh.serviceClient<pendulum::LoggingStart>("/sqlite/start_log");
  if(sql_client.exists()) {
    pendulum::LoggingStart start_log;
    start_log.request.table_name = log_table_name;
    start_log.request.topic_name = name;
    start_log.request.header = header;

    return sql_client.call(start_log) && start_log.response.success;
  }
  return false;
}

void log_trig_functions(ros::NodeHandle &nh)
{
  std::string log_table_name = "ClientTest_Trig";
  std::string topic_name     = "sql_trig_data";
  std::vector<std::string> header({"test_name", "cnt", "radian", "cos", "sin", "tan"});

  // Data publishing object
  ros::Publisher pub = nh.advertise<pendulum::LoggingData>(topic_name, 100);
  if(!start_logging(nh, topic_name, log_table_name, header)){
    ROS_WARN("Logging Start failed: %s.", log_table_name.c_str());
    return;
  }

  ROS_INFO("Beginning SQL client test.");
  ros::Rate rate(25);

  for(size_t i = 0; i < 5; ++i)
    rate.sleep();

  pendulum::LoggingData row;
  std::string trial = "trig_functions";
  std::vector<double> data({0.0, 0.0, 0.0, 0.0, 0.0});

  const double deg2rad = 3.14159265359 / 180.0;
  int count = 0;
  while(ros::ok() && count < 720)
  {
    // First get all updates
    ros::spinOnce();

    row.header.seq += 1;
    row.header.stamp = ros::Time::now();
    row.test_name = trial;

    data[0] = static_cast<double>(count);
    data[1] = count * deg2rad;
    data[2] = cos(data[1]);
    data[3] = sin(data[1]);
    data[4] = tan(data[1]);
    row.data = data;

    pub.publish(row);

    // Sleep until next cycle
    rate.sleep();
    ++count;

    if(count % 50 == 0) {
      ROS_INFO("Trig Logging: Cycle %d", count);
    }
  }

  ROS_INFO("End of Trig Logging");

  // Waits for all data to be inserted into SQLite table
  ros::ServiceClient client_logging_done = nh.serviceClient<pendulum::LoggingBufferCheck>("/sqlite/log_buffer_check");
  while(client_logging_done.exists()) {
    pendulum::LoggingBufferCheck logging_done;
    logging_done.request.table_name = log_table_name;

    if(client_logging_done.call(logging_done)) {
      ros::Duration(0.2).sleep();
      if(logging_done.response.is_empty && logging_done.response.size == 0) { break; }
    } else {
      break;
    }
  }
}

void log_normal_distribution(ros::NodeHandle &nh)
{
    std::string log_table_name = "ClientTest_Normal";
    std::string topic_name     = "sql_normal_data";
    std::vector<std::string> header({"test_name", "i", "x", "y", "z"});

    // Data publishing object
    ros::Publisher pub = nh.advertise<pendulum::LoggingData>(topic_name, 100);
    if(!start_logging(nh, topic_name, log_table_name, header)){
      ROS_WARN("Logging Start failed: %s.", log_table_name.c_str());
      return;
    }

    ROS_INFO("Beginning SQL client test.");
    ros::Rate rate(20);

    for(size_t i = 0; i < 5; ++i)
      rate.sleep();

    std::default_random_engine generator;
    std::normal_distribution<double> distribution(-10.0, 10.0);

    pendulum::LoggingData row;
    int count = 0;
    std::string trial = "trial1";
    std::vector<double> data({0.0, 0.0, 0.0, 0.0});
    while(ros::ok() && count < 500)
    {
      if(count == 49) { trial = "trial2"; }
      // First get all updates
      ros::spinOnce();

      row.header.seq += 1;
      row.header.stamp = ros::Time::now();
      row.test_name = trial;

      data[0] = static_cast<double>(count);
      data[1] = distribution(generator);
      data[2] = distribution(generator);
      data[3] = distribution(generator);
      row.data = data;

      pub.publish(row);

      // Sleep until next cycle
      rate.sleep();
      ++count;

      if(count % 50 == 0) {
        ROS_INFO("Normal Logging: Cycle %d", count);
      }
    }

    ROS_INFO("End of Normal Logging");

    // Waits for all data to be inserted into SQLite table
    ros::ServiceClient client_logging_done = nh.serviceClient<pendulum::LoggingBufferCheck>("/sqlite/log_buffer_check");
    while(client_logging_done.exists()) {
      pendulum::LoggingBufferCheck logging_done;
      logging_done.request.table_name = log_table_name;

      if(client_logging_done.call(logging_done)) {
        ros::Duration(0.2).sleep();
        if(logging_done.response.is_empty && logging_done.response.size == 0) { break; }
      } else {
        break;
      }
    }
}

void test_errors(ros::NodeHandle &nh)
{
  ROS_INFO("Doing nothing for now");
}
