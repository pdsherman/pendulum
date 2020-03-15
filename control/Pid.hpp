/**
  @file:   Pid.hpp
  @author: pdsherman
  @date:   Feb. 2020

  @brief Simple library to implement a PID feedback control algorithm
*/

#pragma once

#include <control/ControlBase.hpp>

class PID : public ControlBase<double, double>
{
public:
  PID(const double delta_t = 1.0,
    const double Kp = 1.0,
    const double Ki = 0.0,
    const double Kd = 0.0,
    const double target = 0.0);

  /// Initialize control loop.
  /// @param [in] x0 Initial value of the state variable
  void init(const double &x0) override;

  /// Perform single cycle of PID control loop.
  /// @param [in] x Current measurement of process variable
  /// @return Updated control value calculated in PID loop
  double update(const double &x) override;

private:

  /// Proportional gain
  double _Kp;

  /// Integral gain
  double _Ki;

  /// Derivative gain
  double _Kd;

  /// Summation of the error over time
  double _integral_sum;

  /// Previous value of process variable
  double _x_prev;

  /// Previous error of last control loop
  double _error_prev;

};
