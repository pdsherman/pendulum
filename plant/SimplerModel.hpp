/*
  File:   Model.hpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Header file for pendulum model for simulation
*/

#pragma once

class SimplerModel
{
public:

  /// Constructor
  /// @param [in] theta0 Initial rotational position
  SimplerModel(double theta0, double b = 0.0, double I = 0.01978);

  /// Default destructor
  ~SimplerModel(void) = default;

  /// Get the current values for the state position values
  /// @return State [x, theta]
  double get_current_state(void) const;

  /// Update the state of the system one time step in the future.
  /// @param [in] h Time step to move system ahead
  void update(const double h);

private:

  /// Calculate the state variable acceleration for a an inverted pendulum with movable base
  /// Uses current values of state and velocity
  /// @return Calculated state acceleration values for x and theta
  double calculate_accel(const double theta, const double velocity) const;

  /// Current position value for pendulum state.
  /// x: Position of movable base along x-axis (meters)
  /// theta: rotational positoin of pendulum (radians)
  double _theta;

  /// Current velocity value for pendulum state
  /// x: Velocity along x-axis (m/s)
  /// theta: Rotational velocity (rad/s)
  double _velocity;

  ///< Rotational friction of pendulum (kg-m^2/s)
  double _b;

  double _I; // = 0.01978; //< Moment of Inertia (kg-m^2)

  // *********************** //
  //   Simulation Constants  //
  // *********************** //

  static constexpr double k = 0.8;
};
