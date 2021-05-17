/*
  @file:    control_base.cpp
  @author:  pdsherman
  @date:    April. 2021

  @brief: control base object
*/


#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingStop.h>
#include <pendulum/LoggingData.h>

#include <pendulum/State.h>
#include <pendulum/Control.h>

#include <plant/BaseObject.hpp>

#include <ros/ros.h>
#include <iostream>
#include <thread>   // For this_thread::sleep_until
#include <chrono>

static constexpr double kRunTime_s    = 5.0;
static constexpr double kMaxInput_N   = 25.0;
static constexpr double kOnTime_s     = 0.5;
static constexpr int kCycleTime_ns = 10000;

std::vector<double> create_command_vector(double input, double on_time, double total_time);

bool start_logging(ros::NodeHandle &nh,
                  const std::string &table,
                  const std::string &topic,
                  const std::vector<std::string> &headers);

void stop_logging(ros::NodeHandle &nh, const std::string &table);


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "base_control");
  ros::NodeHandle nh;

  // ---   Start Logging   --- //
  std::string table = "BaseModelTest";
  std::string topic = "/base_model";
  std::vector<std::string> header({
    "test", "test_time", "input", "x", "x_dot"
  });

  ros::Publisher pub = nh.advertise<pendulum::LoggingData>(topic, 50);
  if(!start_logging(nh, table, topic, header)) {
    ROS_ERROR("Failed to start logging.");
    return 0;
  }

  // Experimentally found that some delay is needed after starting
  // logging before you can start publishing data
  usleep(300000);


  // ---   Run Hardware Test   --- //

  BaseObject::X_t x{0.0, -0.5};
  std::shared_ptr<Plant<2>> plant = BaseObject::create(x);
  std::vector<double> cmd = create_command_vector(kMaxInput_N, kOnTime_s, kRunTime_s);
  int c = 0;
  double t = 0.0;
  double dt = 0.05;
  double u = 0.0;
  while(t < kRunTime_s) {
    // TODO: Run an inner loop at a faster rate than rate updating command

    // Update plant
    u = cmd[c];
    x = plant->update(u, dt);

    // Publish


    t += dt;
    c += 1;
  }

  return 0;
}

std::vector<double> create_command_vector(double input, double on_time, double total_time)
{
  static constexpr double offset = 0.5;
  std::vector<double> vec;

  double t  = 0.0;
  double dt = (kCycleTime_ns/1e9);
  int size = total_time / dt;
  vec.resize(size, 0.0);

  for(size_t i = 0; i < vec.size(); ++i) {
    if(t > offset && t <= on_time + offset) { vec[i] = input; }
    t += dt;
  }

  return vec;
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
