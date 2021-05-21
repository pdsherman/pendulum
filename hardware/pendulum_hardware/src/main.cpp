/*
 * @file    main.cpp
 * @author  pdsherman
 * @date    April 2021
 * @brief   Program for testing complete pendulum hardware
 * @note    This node is intended to run on a separate device than the ROS master and
 *          and other nodes. This runs on RaspberryPi connected to encoder reading board.
 */

#include <pendulum/LoggingData.h>

#include <hardware/pendulum_hardware/PendulumHardware.hpp>
#include <libs/util/FunctionTimer.hpp>
#include <libs/util/ros_util.hpp>

#include <iostream>
#include <thread>   // For this_thread::sleep_until
#include <chrono>
#include <ros/ros.h>

static bool test_request = false;
static std::vector<double> u_cmd;
static std::string test_name;
static const int cycle_time_ns = 1800000; // Target of 1.8 ms per cycle

void hardware_test(ros::NodeHandle &nh, PendulumHardware &hw);

void create_command_vector(double input, double on_time, double total_time);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "pendulum_hardware");
  ros::NodeHandle nh;

  PendulumHardware hw(nh);
  if(!hw.initialize()) {
    ROS_ERROR("Failed to initialize hardware.");
    return 0;
  }
  ROS_INFO("Hardware Initialized. Beginning program...");

  test_name = "Debugging";
  if(argc > 1){
    test_name = std::string(argv[1]);
  }
  create_command_vector(25.0, 0.5, 5.0);


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
  std::string topic = "/base_model";
  std::vector<std::string> header({
    "test", "test_time", "input", "x", "theta"
  });

  ros::Publisher pub = nh.advertise<pendulum::LoggingData>(topic, 50);
  if(!util::start_logging(nh, table, topic, header)) {
    ROS_ERROR("Failed to start logging.");
    return;
  }

  // ---   Run Hardware Test   --- //

  if(!hw.motor_enable()) {
    ROS_ERROR("Error enabling motor.");
  }

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
    auto timestamp = duration_cast<nanoseconds>(steady_clock::now() - start_time);
    msg.data[0] = static_cast<double>(timestamp.count())/(1e9);
    msg.data[1] = u_cmd[i];
    msg.data[2] = pos.position_x;
    msg.data[3] = pos.pendulum_theta;
    msg.header.stamp = ros::Time::now();
    msg.header.seq  += 1;

    pub.publish(msg);
    ros::spinOnce();

    //   Wait until end of cycle time
    std::this_thread::sleep_until(t1);
  }

  hw.motor_disable();

  // ---   Stop Logging   --- //
  util::stop_logging(nh, table);
}

void create_command_vector(double input, double on_time, double total_time)
{
  double t  = 0.0;
  double dt = (cycle_time_ns/1e9);
  int size = total_time / dt;
  u_cmd.resize(size, 0.0);

  double offset = 0.5;

  for(size_t i = 0; i < u_cmd.size(); ++i)
  {
    if(t > offset && t < on_time + offset) {
      u_cmd[i] = input;
    }
    t += dt;
  }

  test_request = true;
}
