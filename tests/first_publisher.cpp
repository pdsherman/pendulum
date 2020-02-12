/*
  File:   first_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Example for simple publisher for c++
*/

#include <iostream>
#include <math.h>
#include <ros/ros.h>

#include <pendulum/AddPendulum.h>
#include <pendulum/State.h>

#define PI 3.14159265

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "first_publisher");
  ros::NodeHandle nh;

  // Initial State
  pendulum::State state;
  state.x = 0.75;
  state.theta = PI/2.0;

  double dt = 0.01;
  double x_dot = 0.1;
  double w = 0.5;
  double t = 0.0;

  // Add pendulum to gui
  pendulum::AddPendulum srv;
  srv.request.name  = "TestPendulum";
  srv.request.x     = state.x;
  srv.request.theta = state.theta;
  srv.request.base_color = "yellow";
  srv.request.pendulum_color = "red";

  ros::ServiceClient client = nh.serviceClient<pendulum::AddPendulum>("/gui/add_pendulum");
  if(client.exists()) {
    client.call(srv);
  } else {
    std::cout << "AddPendulum service doesn't exits. Exiting program.\n";
    return 0;
  }

  // Loop & publish arbitrary state data
  ros::Publisher pub = nh.advertise<pendulum::State>(srv.request.name, 10);
  ros::Rate rate(1/dt);
  while(ros::ok()) {
    state.header.stamp = ros::Time::now();
    state.x += dt*x_dot;
    state.theta = (105.0*sin(w*t) - 90.0)*PI/180.0;
    t += dt;

    if(state.x >= 1.50) {
      state.x = 1.50;
      x_dot = -1.0*fabs(x_dot);
    } else if(state.x <= 0.1) {
      state.x = 0.1;
      x_dot = fabs(x_dot);
    }

    pub.publish(state);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
