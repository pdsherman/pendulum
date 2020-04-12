/*
 * @file    encoder_main.cpp
 * @author  pdsherman
 * @date    March 2020
 * @brief   Node for reading and publishing encoder position values
 * @note    This node is intended to run on a separate device than the ROS master and
 *          and other nodes. This runs on RaspberryPi connected to encoder reading board.
 */

#include <pendulum/State.h>
#include <pendulum/Datapoint.h>
#include <pendulum/AddPendulum.h>
#include <pendulum/StartLogging.h>

#include <devices/Encoder.hpp>

#include <ros/ros.h>

#define PI 3.14159

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "encoder");
  ros::NodeHandle nh;

  // Attempt to Add pendulum to gui. Exit if service server doesn't start up in time.
  ros::ServiceClient gui_client = nh.serviceClient<pendulum::AddPendulum>("/gui/add_pendulum");
  ros::ServiceClient logging_client = nh.serviceClient<pendulum::StartLogging>("/sqlite/start_log");

  int count = 0;
  while((!gui_client.exists() ||!logging_client.exists()) && count < 10) {
    ros::Rate(2).sleep();
    ++count;
  }

  if(count >= 10) {
    ROS_WARN("Timeout waiting for AddPendulum service to exist. Kill Simulation node.");
    return 0;
  }

  pendulum::State state;
  state.x = 1.0;
  state.theta = 0.785;

  // Add pendulum to the GUI
  pendulum::AddPendulum gui_srv;
  gui_srv.request.name  = "encoder";
  gui_srv.request.x     = state.x;
  gui_srv.request.theta = state.theta;
  gui_srv.request.base_color = "blue";
  gui_srv.request.pendulum_color = "#FC33FF";
  gui_client.call(gui_srv);

  // TODO: Remove and place in better position eventually
  pendulum::StartLogging log_srv;
  log_srv.request.table_name = "TestTable";
  log_srv.request.start_time = ros::Time::now();
  logging_client.call(log_srv);

  // State publishing object
  ros::Publisher pub = nh.advertise<pendulum::State>(gui_srv.request.name, 10);

  // Connect to encoder hardware
  Encoder encdr("/dev/i2c-1", 0x28);
  if(!encdr.connect()) {
    ROS_WARN("Couldn't connect to encoder.");
    return 0;
  }
  encdr.zero_position();
  encdr.set_offset(PI/2.0);

  const double dt = 0.02; // Time step
  ros::Rate rate(1/dt);

  while(ros::ok()) {

    state.x = 1.0; 
    state.theta = encdr.position();
    state.header.stamp = ros::Time::now();

    // publish
    pub.publish(state);
    ros::spinOnce();
    rate.sleep();
  }
}
