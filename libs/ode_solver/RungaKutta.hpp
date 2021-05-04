/*
 * @file:   RungaKutta.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Class for general use of 4th order runga-kutta
 */

#pragma once

#include <libs/util/util.hpp>

#include <functional>

/// Use 4th-order Runga-Kutta numerical solver for differential equations to
/// solve second order state equations.
/// i.e. The state equation for X''(t) is known and we want to compute X(t)
template <int N>
class RungaKutta
{
public:

  // Syntax Convenience
  using X_t = std::array<double, N>;

  /// Constructor
  /// @param [in] f Diffential equation for X''. Takes in current state and input command and
  ///               outputs state derivative X'.
  RungaKutta(std::function<X_t(const X_t&, const double)> &&f) : _func(std::move(f))
  {
  }

  /// Default destructor
  ~RungaKutta(void) = default;

  /// Calculate system one time step forward
  /// @param [in] x0 Current state of the system
  /// @param [in] u  Input command at timestamp
  /// @param [in] dt Time step to calculate system forward
  /// @return State of the system one time step forward (numerically solved)
  X_t step(const X_t &x0, const double u, const double dt)
  {
    std::array<std::array<double, N>, 4> kx;

    for(int ii = 0; ii < 4; ++ii) {
      std::array<double, N> delta;
      for(size_t jj = 0; jj < N; ++jj) {
        switch(ii){
          case 0:
            delta[jj] = 0.0;
            break;
          case 1:
            delta[jj] = kx[jj][0]/2.0;
            break;
          case 2:
            delta[jj] = kx[jj][1]/2.0;
            break;
          case 3:
            delta[jj] = kx[jj][2];
            break;
        }
      }

      X_t x_dot = _func(util::add_arrays<double, N>(x0, delta), u);
      for(size_t jj = 0; jj < 4; ++jj)
        kx[jj][ii] = dt*x_dot[jj];
    }

    std::array<double, N> dx;
    for(size_t jj = 0; jj < N; ++jj)
      dx[jj] = (kx[jj][0] + 2*kx[jj][1] + 2*kx[jj][2] + kx[jj][3])/6.0;

    X_t x1 = util::add_arrays<double, N>(x0, dx);

    return x1;
  }

private:

  // State differential equation
  std::function<X_t(const X_t&, const double)> _func;

};
