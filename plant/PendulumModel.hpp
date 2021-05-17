/*
 * File:    PendulumModel.hpp
 * Author:  pdsherman
 * Date:    Aug. 2020
 *
 * Description: Simulate the full pendulum assembly with linear motion
 */

#pragma once

#include <plant/Plant.hpp>

#include <array>
#include <memory>

class PendulumModel : public Plant<4>
{
public:
  using X_t = Plant<4>::X_t;

  /// Constructor
  /// @param x0 initial state [x, x_dot, theta, theta_dot]
  static std::shared_ptr<Plant<4>> create(const X_t &x0);

  /// Constructor
  /// @param x0 initial state [x, x_dot, theta, theta_dot]
  PendulumModel(const X_t &x0);

  /// Destructor
  ~PendulumModel(void) = default;

private:

  /// Calculate the state-variable derivative
  /// @param [in] x Current value of state variable
  /// @param [in] u Value of control variable
  X_t calculate_x_dot(const X_t &x, const double u) const;

  // *********************** //
  //   Simulation Constants  //
  // *********************** //

  static constexpr double l   = 0.5;
  static constexpr double M   = 5.5;
  static constexpr double m   = 2.7;
  static constexpr double b_b = 0.5;
  static constexpr double I   = 0.0105;
  static constexpr double b_p = 0.00078;

  static constexpr double g = 9.80665;  //< Acceleration from gravity (m/s^2)

};
