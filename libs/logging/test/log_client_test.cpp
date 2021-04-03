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
#include <random>
#include <vector>

bool start_logging(ros::NodeHandle &nh, const std::string &name, const std::string &log_table_name)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingDropTable>("/sqlite/drop_table");
  if(sql_client.exists()) {
    pendulum::LoggingDropTable drop_table;
    drop_table.request.table_name = log_table_name;

    sql_client.call(drop_table);
    if(!drop_table.response.success)
      ROS_WARN("Failure to drop table");
  }

  sql_client = nh.serviceClient<pendulum::LoggingStart>("/sqlite/start_log");
  if(sql_client.exists()) {
    pendulum::LoggingStart start_log;
    start_log.request.table_name = log_table_name;
    start_log.request.topic_name = name;
    start_log.request.header = std::vector<std::string>({"trial_name", "num1", "num2", "num3", "num4"});


    return sql_client.call(start_log) && start_log.response.success;
  }
  return false;
}


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "log_client_test");
  ros::NodeHandle nh;

  std::string log_table_name = "ClientTestTable";
  std::string topic_name     = "sql_data";

  // Data publishing object
  ros::Publisher pub = nh.advertise<pendulum::LoggingData>(topic_name, 100);
  if(!start_logging(nh, topic_name, log_table_name)){
    ROS_WARN("Logging Start failed.");
    return 1;
  }

  ROS_INFO("Beginning SQL client test.");
  ros::Rate rate(10);

  for(size_t i = 0; i < 5; ++i)
    rate.sleep();

  std::default_random_engine generator;
  std::normal_distribution<double> distribution(-10.0, 10.0);


  pendulum::LoggingData row;
  int count = 0;
  std::string trial = "trial1";
  std::vector<double> data({0.0, 0.0, 0.0, 0.0});
  while(ros::ok() && count < 100)
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
  }
  ROS_INFO("Ending SQL client test.");


  // Waits for all data to be inserted into SQLite table
  ros::ServiceClient client_logging_done = nh.serviceClient<pendulum::LoggingBufferCheck>("/sqlite/log_buffer_check");
  if(client_logging_done.exists()) {
    while(true) {
      pendulum::LoggingBufferCheck logging_done;
      client_logging_done.call(logging_done);
      ros::Duration(0.2).sleep();
      if(logging_done.response.is_empty && logging_done.response.size == 0) { break; }
    }
  }
  ROS_INFO("Logging Complete");


  return 0;
}
