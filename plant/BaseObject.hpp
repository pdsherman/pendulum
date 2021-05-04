/*
 * File:    BaseObject.hpp
 * Author:  pdsherman
 * Date:    Aug. 2020
 *
 * Description: Simulate the only base object with linear motion
 */

#pragma once

#include <plant/Plant.hpp>

#include <array>
#include <memory>

class BaseObject : public Plant<2>
{
public:
  using X_t = Plant<2>::X_t;

  static std::shared_ptr<Plant<2>> create(const X_t &x0);

  /// Constructor
  /// @param x0 initial state
  BaseObject(const X_t &x0);

  /// Destructor
  ~BaseObject(void) = default;

private:

  /// Calculate the state-variable derivative
  /// @param [in] x Current value of state variable
  /// @param [in] u Value of control variable
  X_t calculate_x_dot(const X_t &x, const double u) const;

  // *********************** //
  //   Simulation Constants  //
  // *********************** //

  static constexpr double M   = 0.617;
  static constexpr double b_b = 40.1;

};
