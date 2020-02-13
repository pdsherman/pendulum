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

class Model
{
public:
  /// Since state already includes x and theta components,
  /// using for velocitys and acclerations. Renaming for readability
  using State = pendulum::State;
  using Velocity = State;
  using Acceleration = State;

  /// Constructor
  /// @param [in] nh ROS node handle object
  /// @param [in] name Identifying name for pendulum object
  /// @param [in] x0 Initial x-axis position of base
  /// @param [in] theta0 Initial rotational position
  Model(ros::NodeHandle &nh, const std::string &name, double x0 = 0.0, double theta0 = 0.0);

  /// Default destructor
  ~Model(void) = default;

  /// Get the current values for the state position values
  /// @return State [x, theta]
  State get_current_state(void) const;

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
  Acceleration calculate_accel(const State &state, const Velocity &velocity) const;

  /// Convenience method to add values to each state variable
  /// @param [in] state Initial state [x, theta]
  /// @param [in] dx Amount to add to x
  /// @param [in] dt Amount to add to theta
  /// @return [state.x + dx, state.theta + dt]
  State add_term(State state, const double dx, const double dt) const;

  /// Callback method for control subscriber to update control value
  /// @param [in] msg Published control message
  void set_control(const pendulum::ControlConstPtr &msg);

  /// Identifying name for object
  std::string _name;

  /// Current position value for pendulum state.
  /// x: Position of movable base along x-axis (meters)
  /// theta: rotational positoin of pendulum (radians)
  State _state;

  /// Current velocity value for pendulum state
  /// x: Velocity along x-axis (m/s)
  /// theta: Rotational velocity (rad/s)
  Velocity _velocity;

  /// Current control input value.
  /// Force (Newtons) exerted on movable base.
  double _u;

  /// Publisher for state value
  ros::Publisher _state_pub;

  /// Subscriber for control value
  ros::Subscriber _control_sub;

  // *********************** //
  //   Simulation Constants  //
  // *********************** //

  static constexpr double m = 1.7; //< mass of pendulum (kilogram)
  static constexpr double M = 0.5; //< mass of base (kilogram)

  static constexpr double l   = 0.25; //< length to midpoint (meters)
  static constexpr double b_x = 1.5;  //< Linear friction for base (kg/s)
  static constexpr double b_t = 0.03; ///< Rotational friction of pendulum (kg/s)
  static constexpr double I   = m*(2*l)*(2*l)/12.0; //< moment of Inertia (kg-m^2)

  static constexpr double g = 9.80665;  //< acceleration from gravity (m/s^2)

};
