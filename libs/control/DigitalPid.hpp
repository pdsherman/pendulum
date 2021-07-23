/**
  @file:   DigitalPid.hpp
  @author: pdsherman
  @date:   May. 2020

  @brief Library to implement a digital PID feedback controller using Euler for digital transform
*/

#pragma once

#include <libs/control/Controller.hpp>


class DigitalPID : public Controller<double, double>
{
public:

  DigitalPID(const double delta_t = 1.0,
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

  double _u_k_1;
  double _u_k_2;
  double _e_k_1;
  double _e_k_2;

};
