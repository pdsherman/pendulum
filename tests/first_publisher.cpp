/*
  File:   first_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Example for simple publisher for c++
*/

#include <iostream>
#include <math.h>
#include <ros/ros.h>

#include <pendulum/State.h>

#define PI 3.14159265

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_publisher");
  ros::NodeHandle nh;

  ros::Rate rate(100);
  ros::Publisher pub = nh.advertise<pendulum::State>("TestPendulum", 10);

  pendulum::State state;
  state.x = 0.75;
  state.theta = PI/2.0;

  double dt = 0.01;
  double x_dot = 0.040;
  double w = 0.2;
  double t = 0.0;

  while(ros::ok()) {
    state.header.stamp = ros::Time::now();
    state.x += dt*x_dot;
    state.theta = -(PI/2.0)*(sin(w*t) + 1.0);
    t += dt;

    if(state.x >= 0.950) {
      state.x = 0.950;
      x_dot = -1.0*fabs(x_dot);
    } else if(state.x <= 0.05) {
      state.x = 0.05;
      x_dot = fabs(x_dot);
    }

    pub.publish(state);
    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}
