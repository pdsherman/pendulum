/*
 * See header file for documentation
 */

#include <libs/control/Pid.hpp>

PID::PID(const double delta_t, const double Kp, const double Ki, const double Kd, const double target)
  : _Kp(Kp), _Ki(Ki), _Kd(Kd),
  _integral_sum(0.0),
  _x_prev(0.0),
  _error_prev(0.0)
{
  _target = target;

  set_delta_time(delta_t);
}

void PID::init(const double &x0)
{
  _u = 0.0;

  _x_prev = x0;
  _integral_sum = 0.0;
  _error_prev = 0.0;
}

double PID::update(const double &x)
{

  double error = _target - x;

  // Proportional
  double out_p = _Kp * error;

  // Integral
  _integral_sum += _dt * (error + _error_prev)/2.0;
  double out_i = _Ki * _integral_sum;

  // Derivative
  double out_d = (_Kd/_dt)*(error - _error_prev);

  // updated control
  double u = out_p + out_i + out_d;

  _error_prev = error;
  _u = u;
  _x_prev = x;

  return u;
}
