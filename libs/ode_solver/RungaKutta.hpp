/*
 * @file:   RungaKutta.hpp
 * @author: pdsherman
 * @date:   April 2020
 * @brief:  Class for general use of 4th order runga-kutta
 */

#pragma once

#include <libs/util/util.hpp>

#include <functional>

/// Numerical ODE solver using Runga-Kutta method
/// Class is setup to system of first order ODEs
/// The solver has been slightly modified from general
/// RK formula as it's intended to be used for simulation
/// of dynamic systems in state space form for control theory.
/// Given:
///     state variables:           x
///     input force:               u
///     time-step:                 dt
///     initial condition:         x0 = x(t0)
///     System of first order ODE: x'(x) = f(x, u)
/// Result of RK method:
///     State 1 step forward:      x1 = x(t0 + dt)
///
/// Explicit RK formula used:
///    x1 = x0 + dt*SUM(b_i*f_i) i=1->s
/// Where
///    f_1 = f(x0, u)
///    f_i = f(x0 + dt*SUM(a_ij*f_j), u) j=1->i-1
///      i = 2, 3,...,s
/// Values of s, b_i, a_ij depend on choice of RK Solver Type
/// and were found in the following book:
///   Title: "Numerical Method for Differential Equations: A Computational Approach"
///  Author: John R. Dormand
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
  /// @param [in] f Diffential equation for x' - x'=f(x, u)
  /// @param [in] type Option for what order of solver and solver coefficients
  RungaKutta(
    std::function<X_t(const X_t&, const double)> &&ode,
    const SolverType type = SolverType::kFourthOrderClassic);

  /// Default destructor
  ~RungaKutta(void) = default;

  /// Calculate system one time step forward
  /// @param [in] x0 Current state of the system - x(t0)
  /// @param [in] u  Input command at current time- u(t0)
  /// @param [in] dt Time step to calculate system forward - dt
  /// @return State of the system one time step forward - x(t0+dt)
  X_t step(const X_t &x0, const double u, const double dt) const;

private:

  // State differential equation. x'(t) = f(x, u)
  std::function<X_t(const X_t&, const double)> _ode_func;

  /// Number of stages. Depends on RK type
  int _s;

  /// Coefficients for RK formula
  /// See comments in step method
  std::vector<std::vector<double>> _a;
  std::vector<double> _b;

};

template<int N>
RungaKutta<N>::RungaKutta(
  std::function<RungaKutta<N>::X_t(const RungaKutta<N>::X_t&, const double)> &&ode,
  const RungaKutta<N>::SolverType type)
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

template<int N>
typename RungaKutta<N>::X_t RungaKutta<N>::step(const RungaKutta<N>::X_t &x0, const double u, const double dt) const
{
  std::vector<X_t> func_evals; // Store evaluation of x' function evaluations f_j
  X_t dx;                      // Delta from RK step (1 for each state variable)

  // Loop number of stages for specified RK type
  for(int i = 0; i < _s; ++i) {
    X_t x_stage = x0; // State to plug into ODE for this stage
    for(int j = 0; j < i; ++j) {
      for(int k = 0; k < N; ++k) {
        if(_a[i][j] != 0.0)
          x_stage[k] += dt*_a[i][j]*func_evals[j][k];
      }
    }

    // Calculate state derivative from ode
    func_evals.push_back(_ode_func(x_stage, u));

    // Add stage function evaluation to dx
    for(int j = 0; j < N; ++j) {
      dx[j] += dt*_b[i]*func_evals[i][j];
    }
  }

  return util::add_arrays<double, N>(x0, dx);
}
