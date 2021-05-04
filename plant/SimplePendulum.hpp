/*
  File:   Model.hpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Class to simulate simple swinging pendulum
*/

#pragma once

#include <plant/Plant.hpp>

#include <array>
#include <memory>

class SimplePendulum : public Plant<4>
{
public:

  using X_t = Plant<4>::X_t;

  static std::shared_ptr<Plant<4>> create(const X_t &x0);

  /// Constructor
  /// @param x0 initial state
  SimplePendulum(const X_t &x0);

  /// Destructor
  ~SimplePendulum(void) = default;

  void set_moment_of_inertia(const double I);

  void set_friction(const double b);


private:

  /// Calculate the state-variable derivative
  /// @param [in] x Current value of state variable
  /// @param [in] u Value of control variable
  X_t calculate_x_dot(const X_t &x, const double u) const;

  ///< Rotational friction of pendulum (kg-m^2/s)
  double _b;

  //< Moment of Inertia (kg-m^2)
  double _I; // = 0.01978;

  // *********************** //
  //   Simulation Constants  //
  // *********************** //

  static constexpr double m = 0.222; //< mass of pendulum (kilogram)
  static constexpr double l = 0.254; //< length to midpoint (meters)
  static constexpr double g = 9.80665;  //< Acceleration from gravity (m/s^2)
};
