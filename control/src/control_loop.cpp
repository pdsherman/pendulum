/*
  @file:    control_loop.cpp
  @author:  pdsherman
  @date:    Feb. 2020

  @brief: Run a PID control loop
*/

#include <pendulum/State.h>
#include <pendulum/Control.h>

#include <control/Pid.hpp>

#include <ros/ros.h>

#include <memory>

using ControlType = ControlBase<double, double>;

double x = 0.0;
bool flag = false;

void set_x(const pendulum::State::ConstPtr &msg)
{
  flag = true;
  x = msg->x;
}

std::shared_ptr<ControlType> control_factory(const int type) {
  std::shared_ptr<ControlType> control = nullptr;

  switch(type) {
    case 1:
      double Kp = 1.0;
      double Ki = 0.1;
      double Kd = 0.0;
      double dt = 0.02;

      control = std::make_shared<ControlType>(new PID(Kp, Ki, Kd));
      control->set_delta_time(dt);

      break;
  }

  return control;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "PendulumControl");
  ros::NodeHandle nh;

  ros::Publisher pub  = nh.advertise<pendulum::Control>("control", 50);
  ros::Subscriber sub = nh.subscribe("Simulation", 50, &set_x);

  pendulum::Control u;
  u.header.seq = 0;
  u.u = 0.0;

  ros::Rate rate(50);

  while(!flag)
    rate.sleep();

  std::shared_ptr<ControlType> control = control_factory(1);
  control->set_target(2.0);
  control->init(x);

  while(ros::ok())
  {
    ros::spinOnce();

    u.header.seq += 1;
    u.header.stamp = ros::Time::now();
    u.u = control->update(x);

    pub.publish(u);
    rate.sleep();
  }


}
