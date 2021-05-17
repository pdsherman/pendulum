/*
 * @file:   RungaKutta.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Class for general use of 4th order runga-kutta
 */

#pragma once

#include <libs/util/util.hpp>

#include <functional>

/// Use Runga-Kutta numerical solver for differential equations to
/// solve second order state equations by solving as system of
/// first order state equations.
/// Given:
///     state variables x
///     x''(x) = f(x, x', u)  and  x(x0) = x0
template <int N>
class RungaKutta
{
public:

  /// Options for RK solver
  enum class SolverType {
    kThirdOrder,
    kFourthOrderClassic,
    kFourthOrderOptimal
  };

  // Syntax Convenience
  using X_t = std::array<double, N>;

  /// Constructor
  /// @param [in] f Diffential equation for X''. Takes in current state and input command and
  ///               outputs state derivative X'.
  RungaKutta(std::function<X_t(const X_t&, const double)> &&ode, const SolverType type = SolverType::kFourthOrderClassic)
   : _ode_func(std::move(ode))
  {
    switch(type) {
      case SolverType::kThirdOrder:
        _s = 3;
        _b = {1.0/6.0, 2.0/3.0, 1.0/6.0};

        _a.resize(3, std::vector<double>(2, 0.0));
        _a[1][0] =  0.5;
        _a[2][0] = -1.0;
        _a[2][1] =  2.0;
        break;
      case SolverType::kFourthOrderClassic:
        _s = 4;
        _b = {1.0/6.0, 1.0/3.0, 1.0/3.0, 1.0/6.0};

        _a.resize(4, std::vector<double>(3, 0.0));
        _a[1][0] = 0.5;
        _a[2][1] = 0.5;
        _a[3][2] = 1.0;
        break;
      case SolverType::kFourthOrderOptimal:
        _s = 4;
        _b = {0.17476028, -0.55148066, 1.20553560, 0.17118478};

        _a.resize(4, std::vector<double>(3, 0.0));
        _a[1][0] =  0.4;
        _a[2][0] =  0.29697761;
        _a[2][1] =  0.15875964;
        _a[3][0] =  0.21810040;
        _a[3][1] = -3.05096516;
        _a[3][2] =  3.83286476;
        break;
    }
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
    // Store evaluation of x' function of each state variable, for each stage
    std::vector<X_t> x_dot_evals;

    // Delta of each state variable from RK step
    X_t dx;

    // Loop number of stage for specified RK type
    for(int i = 0; i < _s; ++i) {
      X_t x_stage = x0; // State to plug into ODE for this stage
      for(int j = 0; j < i; ++j) {
        for(int k = 0; k < N; ++k)
          x_stage[k] += dt*_a[i][j]*x_dot_evals[j][k];
      }

      // Calculate state derivative from ode
      x_dot_evals.push_back(_ode_func(x_stage, u));

      // Add stage function evaluation to dx
      for(int j = 0; j < N; ++j) {
        dx[j] += dt*_b[i]*x_dot_evals[i][j];
      }
    }

    return util::add_arrays<double, N>(x0, dx);
  }

private:

  // State differential equation
  std::function<X_t(const X_t&, const double)> _ode_func;

  /// Number of stages
  int _s;

  /// Coefficients for RK formula
  std::vector<std::vector<double>> _a;
  std::vector<double> _b;

};
