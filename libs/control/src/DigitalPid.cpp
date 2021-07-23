/*
 * See header file for documentation
 */

#include <libs/control/DigitalPid.hpp>

DigitalPID::DigitalPID(const double delta_t, const double Kp, const double Ki, const double Kd, const double target)
  : _Kp(Kp), _Ki(Ki), _Kd(Kd),
  _u_k_1(0.0), _u_k_2(0.0), _e_k_1(0.0), _e_k_2(0.0)
{
  _target = target;
  set_delta_time(delta_t);
}

void DigitalPID::init(const double &x0)
{
  _u_k_1 = 0.0;
  _u_k_2 = 0.0;
  _e_k_1 = 0.0;
  _e_k_2 = 0.0;
}

double DigitalPID::update(const double &x)
{
  double err = _target - x;
  double u = _u_k_2;
  u += _Kp*(err - _e_k_2);
  u += (_dt*_Ki/2.0)*(err + (2.*_e_k_1) + _e_k_2);
  u += (2.0*_Kd/_dt)*(err - (2.*_e_k_1) + _e_k_2);

  _e_k_2 = _e_k_1;
  _e_k_1 = err;

  _u_k_2 = _u_k_1;
  _u_k_1 = u;

  _u = u;
  return _u;
}
