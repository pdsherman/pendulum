/**
  @file:   LeadLag.hpp
  @author: pdsherman
  @date:   May. 2020

  @brief Library to implement a digital LeadLag controller using Euler for digital transform
*/

#pragma once

#include <libs/control/Controller.hpp>

#include <array>

/// Implement digital controller for lead/lag compensator
/// Approximates the following continuous transform function:
///
///            s + z
///   D(s) = K -----
///            s + p
///
/// The digital transformation results in the following
/// update equation:
///       u(k) = c1*u(k-1) + c2*e(k) + c3*e(k-1)
/// Coefficients determined:
///   - Digital transform method
///   - Values of K, z & p
///   - Sample time T
class LeadLag : public Controller<double, double>
{
public:

  enum class DigitialTransform {
    kTustins,
    kEuler,
    kBackwordDiff
  };

  LeadLag(const double T, const double z, const double p, const double K,
    const DigitialTransform method = DigitialTransform::kTustins);

  /// Initialize control loop.
  /// @param [in] x0 Initial value of the state variable
  void init(const double &x0) override;

  /// Perform single cycle for control loop according to following:
  /// @param [in] x Current measurement of process variable
  /// @return Updated control value from controller
  double update(const double &x) override;

private:

  double _z;
  double _p;
  double _K;

  /// Coefficients for control update equation
  std::array<double, 3> _c;

  double _error;

};
