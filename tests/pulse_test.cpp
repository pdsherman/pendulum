/*
  File:   pulse_test.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Publish a single pulse to control topic.
*/

#include <pendulum/Control.h>
#include <ros/ros.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "PulsePublisher");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<pendulum::Control>("control", 50);

  pendulum::Control ctrl;
  ctrl.u = 0.0;
  ctrl.header.seq = 0;

  double t = 0;
  double dt = 0.05;

  ros::Rate rate(1/dt);
  while(ros::ok())
  {
    ctrl.header.seq += 1;
    ctrl.header.stamp = ros::Time::now();

    if( t >= 5.0 && t < 5.2) {
      ctrl.u = 10.0;
    } else {
      ctrl.u = 0.0;
    }

    t += dt;
    pub.publish(ctrl);
    rate.sleep();
  }

  return 0;
}
