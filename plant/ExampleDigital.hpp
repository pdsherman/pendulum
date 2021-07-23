/*
 * File:    BaseObject.hpp
 * Author:  pdsherman
 * Date:    Aug. 2020
 *
 * Description: Simulate the only base object with linear motion
 */

#pragma once

#include <plant/Plant.hpp>

#include <memory>

class ExampleDigital : public Plant<2>
{
public:
  using X_t = Plant<2>::X_t;

  static std::shared_ptr<Plant<2>> create(const X_t &x0);

  /// Constructor
  /// @param x0 initial state
  ExampleDigital(const X_t &x0);

  /// Destructor
  ~ExampleDigital(void) = default;

private:

  /// Calculate the state-variable derivative
  /// @param [in] x Current value of state variable
  /// @param [in] u Value of control variable
  X_t calculate_x_dot(const X_t &x, const double u) const;

};
