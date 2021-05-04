/*
  @file:    control_base.cpp
  @author:  pdsherman
  @date:    April. 2021

  @brief: control base object
*/


#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingStop.h>
#include <pendulum/LoggingData.h>

#include <pendulum/State.h>
#include <pendulum/Control.h>

#include <plant/Plant.hpp>
#include <plant/PendulumModel.hpp>
#include <plant/BaseObject.hpp>
#include <libs/ode_solver/RungaKutta.hpp>

#include <ros/ros.h>
#include <iostream>
#include <thread>   // For this_thread::sleep_until
#include <chrono>
#include <functional>

using X_t = PendulumModel::X_t;

X_t calc(const X_t &x, const double u)
{
  static constexpr double l   = 0.5;
  static constexpr double M   = 5.5;
  static constexpr double m   = 2.7;
  static constexpr double b_b = 0.5;
  static constexpr double I   = 0.00474;
  static constexpr double b_p = 0.00061;
  static constexpr double g   = 9.80665;  //< Acceleration from gravity (m/s^2)

  static constexpr double d = M + m;
  static constexpr double k = m*l;
  static constexpr double w = b_p/l;
  static constexpr double z = (m*l) + (I/l);

  const double c2 = cos(x[2]);
  const double s2 = sin(x[2]);
  const double s2_sqr = pow(s2, 2.0);

  X_t x_dot;

  // x-dot
  x_dot[0] = x[1];

  // x-double-dot
  const double n1 = z / (d*z - m*k*s2_sqr);
  const double n2 = (2.0*k + z) / z;
  x_dot[1] = n1*(-b_b*x[1] + k*c2*pow(x[3], 2.0) - n2*w*s2*x[3] - m*k*g*sin(2*x[2])/(2*z) + u);

  // theta-dot
  x_dot[2] = x[3];

  // theta-double-dot
  const double n3 = m / (d*z - m*k*s2_sqr);
  const double n4 = 0.5*k*sin(2.0*x[2]);
  const double n5 = (2.0*d + m*s2_sqr) / m;
  x_dot[3] = n3*(-b_b*s2*x[1] + n4*pow(x[3], 2.0) - n5*w*x[3] - d*g*c2 + s2*u);

  return x_dot;
}

int main(int argc, char *argv[])
{


//  ros::init(argc, argv, "base_control");
//  ros::NodeHandle nh;

  double dt = 0.05;
  double u  = 10.0;
  PendulumModel::X_t x0{0.1, -0.5, 2.3, 5.0};

  PendulumModel m(x0);

  std::function<X_t(const X_t&, const double)> f = &calc;
  RungaKutta<4> rk(std::move(f));

  X_t x_1 = rk.step(x0, u, dt);
  X_t x_2 = m.update(u, dt);
  std::cout << "1: " << util::array_to_string<double, 4>(x_1) << std::endl;
  std::cout << "2: " << util::array_to_string<double, 4>(x_2) << std::endl;

  return 0;
}
