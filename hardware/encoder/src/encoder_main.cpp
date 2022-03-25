/*
 * @file    encoder_main.cpp
 * @author  pdsherman
 * @date    March 2020
 * @brief   Node for reading and publishing encoder position values
 * @note    This node is intended to run on a separate device than the ROS master and
 *          and other nodes. This runs on RaspberryPi connected to encoder reading board.
 */

#include <pendulum/State.h>
#include <pendulum/DrawSystem.h>
#include <pendulum/LoggingStart.h>

#include <hardware/encoder/EncoderBoard.hpp>

#include <ros/ros.h>

#define PI 3.14159

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "single_encoder");
  ros::NodeHandle nh;

  // Attempt to Add pendulum to gui. Exit if service server doesn't start up in time.
  ros::ServiceClient gui_client = nh.serviceClient<pendulum::DrawSystem>("/gui/add_pendulum");
  //ros::ServiceClient logging_client = nh.serviceClient<pendulum::LoggingStart>("/sqlite/start_log");

  int count = 0;
  while(!gui_client.exists() && count < 100) {
    ros::Rate(2).sleep();
    ++count;
  }

  if(count >= 10) {
    ROS_WARN("Timeout waiting for DrawSystem service to exist. Kill Simulation node.");
    return 0;
  }

  pendulum::State state;
  state.x = 1.0;
  state.theta = 0.785;

  // Add pendulum to the GUI
  pendulum::DrawSystem gui_srv;
  gui_srv.request.name  = "Encoder";
  gui_srv.request.x     = state.x;
  gui_srv.request.theta = state.theta;
  gui_srv.request.base_color = "blue";
  gui_srv.request.pendulum_color = "#FC33FF";
  gui_srv.request.img_type = pendulum::DrawSystemRequest::PENDULUM;
  gui_client.call(gui_srv);


  // State publishing object
  ros::Publisher pub = nh.advertise<pendulum::State>(gui_srv.request.name, 10);

  // Connect to encoder hardware
  EncoderBoard encdr("/dev/i2c-1", 0x33);
  if(!encdr.connect()) {
    ROS_WARN("Couldn't connect to encoder.");
    return 0;
  }

  encdr.set_mode(EncoderBoard::Mode::Both);
  encdr.zero_position();
  encdr.set_offset(EncoderBoard::Encoder::One, -PI/2.0);

  const double dt = 0.02; // Time step
  ros::Rate rate(1/dt);

  ROS_INFO("Starting Loop");
  while(ros::ok()) {
    // update state variable
    std::array<double, 2> pos = encdr.position();
    state.x = 1.0 + 0.02*PI*pos[1];
    state.theta = pos[0];
	  state.header.seq += 1;
    state.header.stamp = ros::Time::now();

    // publish and delay
    pub.publish(state);
    ros::spinOnce();
    rate.sleep();
  }
}
