/*
  File:   model_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Physics based simulation for inverted pendulum
*/

#include <pendulum/State.h>
#include <pendulum/AddPendulum.h>

#include <plant/Model.hpp>

#include <ros/ros.h>
#include <iostream>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Simulation");
  ros::NodeHandle nh;

  double x0     = 1.00;
  double theta0 = 89.0*3.14/180.0;
  std::string name = "Simulated";

  Model pendulum(nh, name, x0, theta0);

  // Add pendulum to gui
  pendulum::AddPendulum srv;
  srv.request.name  = name;
  srv.request.x     = x0;
  srv.request.theta = theta0;
  srv.request.base_color = "blue";
  srv.request.pendulum_color = "#FC33FF";

  ros::ServiceClient client = nh.serviceClient<pendulum::AddPendulum>("/gui/add_pendulum");
  if(client.exists()) {
    client.call(srv);
  } else {
    std::cout << "AddPendulum service doesn't exits. Exiting program.\n";
    return 0;
  }

  // Loop & publish simulated state data
  const double dt = 0.02; // Time step
  ros::Rate rate(1/dt);
  while(ros::ok())
  {
    // First get all updates
    ros::spinOnce();

    // Simulate system one time step forward.
    pendulum.update(dt);
    pendulum.publish(ros::Time::now());

    // Sleep until next cycle
    rate.sleep();
  }

  return 0;
}
