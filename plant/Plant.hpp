/*
 * File:    Plant.hpp
 * Author:  pdsherman
 * Date:    Aug. 2020
 *
 * Description: Abstract base class for plants to to use in project
 */

#pragma once

#include <libs/ode_solver/RungaKutta.hpp>

#include <array>

template <int N>
class Plant
{
public:
  using X_t = std::array<double, N>;

  /// Constructor
  /// @param [in] x0 initial state
  /// @param [in] f  Function to define different equation of plant
  Plant(const X_t &x0, std::function<X_t(const X_t&, const double)> &&f)
    : _x(x0),
      _solver(std::move(f))
  {
  }

  /// Destructor
  virtual ~Plant(void) = default;

  /// Simulate system one timestep in future
  /// @param [in] u Control during time step
  /// @param [in] dt Time-step to simulate
  virtual X_t update(const double u, const double dt)
  {
    _x = _solver.step(_x, u, dt);
    return _x;
  }
  /// Get the current state of the system
  /// @return Value of state variables as array
  X_t get_x(void) const
  {
    return _x;
  }

protected:
  /// Current value of state evariables
  X_t _x;

  /// Numerical solver for second order differetial solver
  ///   X''=f(x, x', u)
  RungaKutta<N> _solver;

};
