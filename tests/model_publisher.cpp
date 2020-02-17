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
#include <ros/console.h>


int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Simulation");
  ros::NodeHandle nh;

  // Initial conditions
  double x0     = 0.2;
  double theta0 = 1.3;
  std::string name = "Simulated";

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

  pendulum::AddPendulum srv;
  srv.request.name  = name;
  srv.request.x     = x0;
  srv.request.theta = theta0;
  srv.request.base_color = "blue";
  srv.request.pendulum_color = "#FC33FF";
  client.call(srv);

  // Run simulation & publish state data
  const double dt = 0.02; // Time step
  ros::Rate rate(1/dt);

  Model pendulum(nh, name, x0, theta0);
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
