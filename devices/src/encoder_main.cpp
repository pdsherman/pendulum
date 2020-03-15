/*
 * @file    encoder_main.cpp
 * @author  pdsherman
 * @date    March 2020
 * @brief   Node for reading and publishing encoder position values
 */

#include <pendulum/State.h>
#include <pendulum/AddPendulum.h>

#include <devices/Encoder.hpp>

#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Encoder");
  ros::NodeHandle nh;

  // Attempt to Add pendulum to gui. Exit if servicer server doesn't start up in time.
  ros::ServiceClient client = nh.serviceClient<pendulum::AddPendulum>("/gui/add_pendulum");
  int count = 0;
  while(!client.exists() && count < 10) {
    ros::Rate(2).sleep();
    ++count;
  }

  if(count >= 10) {
    ROS_WARN("Timeout waiting for AddPendulum service to exist. Kill Simulation node.");
    return 0;
  }

  pendulum::State state;
  state.x = 1.5;
  state.theta = 0.785;

  pendulum::AddPendulum srv;
  srv.request.name  = "Encoder";
  srv.request.x     = state.x;
  srv.request.theta = state.theta;
  srv.request.base_color = "blue";
  srv.request.pendulum_color = "#FC33FF";
  client.call(srv);

  ros::Publisher pub = nh.advertise<pendulum::State>(srv.request.name, 10);

  Encoder encdr("/dev/i2c-1", 0x28);
  if(!encdr.connect()) {
    ROS_WARN("Couldn't connect to encoder.");
    return 0;
  }

  encdr.zero_position();

  const double dt = 0.02; // Time step
  ros::Rate rate(1/dt);
  while(ros::ok()) {

    double pos = static_cast<double>(encdr.position());
    double rad = 3.14159*(pos/90.0 + 0.25);
    state.theta = rad;
    state.header.stamp = ros::Time::now();

    // publish
    pub.publish(state);
    ros::spinOnce();
    rate.sleep();
  }
}
