/*
 * @file    main.cpp
 * @author  pdsherman
 * @date    April 2021
 * @brief   Program for testing complete pendulum hardware
 * @note    This node is intended to run on a separate device than the ROS master and
 *          and other nodes. This runs on RaspberryPi connected to encoder reading board.
 */

#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingStop.h>
#include <pendulum/LoggingData.h>

#include <hardware/pendulum_hardware/PendulumHardware.hpp>
#include <libs/function_timer/FunctionTimer.hpp>

#include <iostream>
#include <thread>   // For this_thread::sleep_until
#include <chrono>
#include <ros/ros.h>

static bool test_request = false;
static std::vector<double> u_cmd;
static std::string test_name;
static const int cycle_time_ns = 1800000; // Target of 1.8 ms per cycle


void hardware_test(ros::NodeHandle &nh, PendulumHardware &hw);

bool start_logging(ros::NodeHandle &nh,
                  const std::string &table,
                  const std::string &topic,
                  const std::vector<std::string> &headers);

void stop_logging(ros::NodeHandle &nh, const std::string &table);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pendulum_hardware");
  ros::NodeHandle nh;

  PendulumHardware hw(nh);
  if(!hw.initialize()) {
    ROS_ERROR("Failed to initialize hardware.");
    return -1;
  }
  ROS_INFO("Hardware Initialized. Begining program...");
  std::string test_name = "BaseTest";

  while(ros::ok()) {
    if(test_request) {
        ROS_INFO("Hardware Test Request");
        hardware_test(nh, hw);
    }
    test_request = false;
    ros::Duration(0.1).sleep();
  }

  ROS_INFO("...Hardware Program Completed");
  return 0;
}

void hardware_test(ros::NodeHandle &nh, PendulumHardware &hw)
{
  using namespace std::chrono;

  // ---   Start Logging   --- //
  std::string table = "BaseModelTest";
  std::string topic = "";
  std::vector<std::string> header({
    "test", "test_time", "input", "x", "theta"
  });

  if(!start_logging(nh, table, topic, header)) {
    ROS_ERROR("Failed to start logging.");
    return;
  }

  // ---   Run Hardware Test   --- //
  pendulum::LoggingData msg;
  msg.test_name = test_name;
  msg.data = std::vector<double>(4);

  steady_clock::time_point start_time = steady_clock::now();
  steady_clock::time_point t1 = start_time;
  for(size_t i = 0; i < u_cmd.size(); ++i) {
    //   Calculate end of cycle time
    t1 += nanoseconds(cycle_time_ns);

    // Update hardware
    PendulumHardware::Positions pos = hw.update(u_cmd[i]);

    //   Log data
    auto timestamp = duration_cast<milliseconds>(steady_clock::now() - start_time);
    msg.data[0] = static_cast<double>(timestamp.count())/1000.0;
    msg.data[1] = u_cmd[i];
    msg.data[2] = pos.position_x;
    msg.data[3] = pos.pendulum_theta;
    msg.header.stamp = ros::Time::now();
    msg.header.seq  += 1;
    ros::spinOnce();

    //   Wait until end of cycle time
    std::this_thread::sleep_until(t1);
  }

  // ---   Stop Logging   --- //
  stop_logging(nh, table);
}

bool start_logging(ros::NodeHandle &nh,
                  const std::string &table,
                  const std::string &topic,
                  const std::vector<std::string> &header)
{
  // Request logging node to start logging
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingStart>("/sqlite/start_log");
  if(sql_client.exists()) {
    pendulum::LoggingStart start_log;
    start_log.request.table_name = table;
    start_log.request.topic_name = topic;
    start_log.request.header     = header;

    return sql_client.call(start_log) && start_log.response.success;
  } else {
    ROS_WARN("Start logging server unreachable.");
  }

  return false;
}

void stop_logging(ros::NodeHandle &nh, const std::string &table)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingStop>("/sqlite/stop_log");
  if(sql_client.exists()) {
    pendulum::LoggingStop start_log;
    start_log.request.table_name = table;
    sql_client.call(start_log);
  } else {
    ROS_WARN("Stop logging server unreachable.");
  }
}
