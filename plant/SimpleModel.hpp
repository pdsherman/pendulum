/*
  File:   Model.hpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Header file for pendulum model for simulation
*/

#pragma once

#include <pendulum/State.h>
#include <pendulum/Control.h>

#include <ros/ros.h>

class SimpleModel
{
public:

  /// Constructor
  /// @param [in] nh ROS node handle object
  /// @param [in] name Identifying name for pendulum object
  /// @param [in] theta0 Initial rotational position
  SimpleModel(ros::NodeHandle &nh, const std::string &name, double theta0, double b = 0.0, double I = 0.01978);

  /// Default destructor
  ~SimpleModel(void) = default;

  /// Get the current values for the state position values
  /// @return State [x, theta]
  double get_current_state(void) const;

  /// Publish current state
  /// @param [in] time Time to use for header timestamp
  void publish(const ros::Time &time);

  /// Update the state of the system one time step in the future.
  /// @param [in] h Time step to move system ahead
  void update(const double h);

private:

  /// Calculate the state variable acceleration for a an inverted pendulum with movable base
  /// Uses current values of state and velocity
  /// @return Calculated state acceleration values for x and theta
  double calculate_accel(const double theta, const double velocity) const;

  /// Identifying name for object
  std::string _name;

  /// Current position value for pendulum state.
  /// x: Position of movable base along x-axis (meters)
  /// theta: rotational positoin of pendulum (radians)
  double _theta;

  /// Current velocity value for pendulum state
  /// x: Velocity along x-axis (m/s)
  /// theta: Rotational velocity (rad/s)
  double _velocity;

  /// Publisher for state value
  ros::Publisher _state_pub;

  /// Subscriber for control value
  ros::Subscriber _control_sub;

  ///< Rotational friction of pendulum (kg-m^2/s)
  double _b;

  double _I; // = 0.01978; //< Moment of Inertia (kg-m^2)

  // *********************** //
  //   Simulation Constants  //
  // *********************** //

  static constexpr double m = 0.221; //< mass of pendulum (kilogram)
  static constexpr double l = 0.25; //< length to midpoint (meters)
  //static constexpr double I = 0.01978; //< Moment of Inertia (kg-m^2)
  static constexpr double g = 9.80665;  //< Acceleration from gravity (m/s^2)
};
