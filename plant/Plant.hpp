/*
 * File:    Plant.hpp
 * Author:  pdsherman
 * Date:    Aug. 2020
 *
 * Description: Abstract base class for plants to to use in project
 */

#pragma once

#include <array>

template <int N>
class Plant
{
public:
  using State = std::array<double, N>;

  /// Constructor
  /// @param x0 initial state
  Plant(const State &x0) : _x(x0)
  {
  }

  /// Destructor
  virtual ~Plant(void) = default;

  /// Simulate system one timestep in future
  /// @param u Control during time step
  /// @param dt Time-step to simulate
  virtual State update(const double u, const double dt) = 0;

  /// Get the current state of the system
  /// @return Value of state variables as array
  State get_x(void) const
  {
    return _x;
  }

protected:
  /// Current value of state evariables
  State _x;
};
