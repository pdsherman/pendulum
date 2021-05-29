/*
 * See header file for documentation
 */

#include <libs/control/DigitalPid.hpp>

DigitalPID::DigitalPID(const double delta_t, const double Kp, const double Ki, const double Kd, const double target)
  : _Kp(Kp), _Ki(Ki), _Kd(Kd),
  _u_prev_1(0.0), _u_prev_2(0.0),
  _error_prev_1(0.0), _error_prev_2(0.0)
{
  _target = target;

  set_delta_time(delta_t);
}

void DigitalPID::init(const double &x0)
{
  _u = 0.0;
  _u_prev_1 = 0.0;
  _u_prev_2 = 0.0;
  _error_prev_1 = 0.0;
  _error_prev_2 = 0.0;
}

double DigitalPID::update(const double &x)
{
  double error = _target - x;

  double prop = _Kp*(error - _error_prev_2);
  double deri = (2*_Kd/_dt)*(error - 2*_error_prev_1 + _error_prev_2);
  double inte = (0.5*_dt*_Ki)*(error + 2*_error_prev_1 + _error_prev_2);
  double u = _u_prev_2 + prop + deri + inte;

  _u_prev_2 = _u_prev_1;
  _u_prev_1 = _u;
  _u = u;

  _error_prev_2 = _error_prev_1;
  _error_prev_1 = error;

  return u;
}
