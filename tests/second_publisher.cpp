/*
  File:   second_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Physics based simulation for inverted pendulum
*/

#include <pendulum/State.h>
#include <pendulum/AddPendulum.h>

#include <ros/ros.h>
#include <iostream>
#include <array>
#include <cmath>

// *********************** //
//   Simulation Constants  //
// ********************** //

static constexpr double PI = 3.14159265;
static constexpr double g  = 9.80665;  // m/s^2

static constexpr double m = 1.7; // kg
static constexpr double M = 0.5; // kg

static constexpr double l = 0.25;  // m, length to midpoint
static constexpr double b = 1.5;  // kg/s
static constexpr double r = 0.05;  // kg/s
static constexpr double I = m*(2*l)*(2*l)/12.0;   // kg-m^2

// ******************* //
//   Local Functions   //
// ******************* //

// Since state already includes x and theta components,
// using for velocitys and acclerations. Renaming for readability
using State = pendulum::State;
using Velocity = State;
using Acceleration = State;

// Calculate the state variable acceleration for a an inverted pendulum with movable base
// Using the state variables: [x, theta]
// @param [in] state The state position values
// @param [in] velocity The state velocity values
// @param [in] u Input force on the base
// @return Calculated state acceleration values
State ddxddt(const State &state, const Velocity &velocity, double u = 0.0);

// Calculate the numerical solution for the system of equations for the inverted pendulum
// using 4th order Runga-Kutta method.
// @param [in] x0 Current state position values: x(t)
// @param [in] v0 Current state velocity values: v(t)
// @param [out] x1 Calculated state position one step forward: x(t + h)
// @param [out] v1 Calculated state velocity one step forward: v(t + h)
// @param [in] u Input control to use during time step
// @param [in] h Time step to move system ahead
void runga_kutta(
  const State &x0, const Velocity &v0, State &x1, Velocity &v1, const double u, const double h);

// Convenience method to add values to each state variable
// @param [in] state Initial state [x, theta]
// @param [in] dx Amount to add to x
// @param [in] dt Amount to add to theta
// @return [state.x + dx, state.theta + dt]
State add_term(State state, const double dx, const double dt);

// ******************* //
//        MAIN         //
// ******************* //
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "test_model");
  ros::NodeHandle nh;

  // Ititial State
  State x0;
  x0.x = 0.75;
  x0.theta = 15.0*PI/180.0;

  // Initial Velocity
  Velocity v0;
  v0.x = 0.0;
  v0.theta = 0.0;

  // Time step
  double dt = 0.02;
  double t = 0.0;

  // State and Velocity for next time-step
  State x1;
  Velocity v1;


  // Add pendulum to gui
  pendulum::AddPendulum srv;
  srv.request.name  = "Simulated";
  srv.request.x     = x0.x;
  srv.request.theta = x0.theta;
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
  ros::Rate rate(1/dt);
  ros::Publisher pub = nh.advertise<pendulum::State>(srv.request.name, 10);
  while(ros::ok())
  {
    // x1 and x1_dot will be updated in runga_kutta
    runga_kutta(x0, v0, x1, v1, 0.0, dt);

    x0.header.stamp = ros::Time::now();
    x0 = x1;
    v0 = v1;

    t += dt;
    pub.publish(x0);
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}

State ddxddt(const State &state, const Velocity &velocity, double u)
{
  Acceleration accel;

  // Constant values used in calculations
  static constexpr double m_M  = m + M;
  static constexpr double ll   = l*l;
  static constexpr double ml   = m*l;
  static constexpr double I_ = I + ll;

  // Calculate acceleration of x
  double k1 = -b*velocity.x;
  double k2 = l*pow(velocity.x, 2.0)*cos(state.theta);
  double k3 = -m*ll*g*sin(state.theta)*cos(state.theta)/I_ + u;
  double k4 = m_M - m*ll*pow(sin(state.theta), 2.0)/I_;
  accel.x = (k1 + k2 + k3)/k4;

  // Calculate rotational acceleration of theta
  double k5 = ml/I_;
  double k6 = accel.x*sin(state.theta) - g*cos(state.theta) - r*velocity.theta;
  accel.theta = k5*k6;

  return accel;
}

void runga_kutta(
  const State &x0, const Velocity &v0, State &x1, Velocity &v1, const double u, const double h)
{
  std::array<double, 4> kx, kt, fx, ft;

  for(int ii = 0; ii < 4; ++ii) {
    double dx, dt, ddx, ddt;
    switch(ii){
      case 0:
        dx  = 0.0;
        dt  = 0.0;
        ddx = 0.0;
        ddt = 0.0;
      case 1:
        dx  = kx[0]/2.0;
        dt  = kt[0]/2.0;
        ddx = fx[0]/2.0;
        ddt = ft[0]/2.0;
        break;
      case 2:
        dx  = kx[1]/2.0;
        dt  = kt[1]/2.0;
        ddx = fx[1]/2.0;
        ddt = ft[1]/2.0;
        break;
      case 3:
        dx  = kx[2];
        dt  = kt[2];
        ddx = fx[2];
        ddt = ft[2];
        break;
    }

    Acceleration accel = ddxddt(add_term(x0, dx, dt), add_term(v0, ddx, ddt), u);
    fx[ii] = h*accel.x;
    ft[ii] = h*accel.theta;

    kx[ii] = h*(v0.x + ddx);
    kt[ii] = h*(v0.theta + ddt);
  }

  // Combine and update state 1
  double dx     = (kx[0] + 2*kx[1] + 2*kx[2] + kx[3])/6.0;
  double dtheta = (kt[0] + 2*kt[1] + 2*kt[2] + kt[3])/6.0;

  double dx_dot     = (fx[0] + 2*fx[1] + 2*fx[2] + fx[3])/6.0;
  double dtheta_dot = (ft[0] + 2*ft[1] + 2*ft[2] + ft[3])/6.0;

  x1 = add_term(x0, dx, dtheta);
  v1 = add_term(v0, dx_dot, dtheta_dot);
}

State add_term(State state, const double dx, const double dt)
{
  state.x += dx;
  state.theta += dt;
  return state;
}
