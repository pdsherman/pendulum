/*
 * See header file for documentation
 */

#include <libs/control/LeadLag.hpp>

LeadLag::LeadLag(const double T, const double z, const double p, const double K,
  const DigitialTransform method):
  _z(z), _p(p), _K(K)
{
  set_delta_time(T);

  double n = 0.0;
  switch (method) {
    case DigitialTransform::kTustins:
      n = _p*_dt + 2.0;
      _c[0] = (2.0 - _p*_dt)/n;
      _c[1] = _K*(_z*_dt + 2.0)/n;
      _c[2] = _K*(_z*_dt - 2.0)/n;
      break;
    case DigitialTransform::kEuler:
      _c[0] = (1 - _p*_dt);
      _c[1] = _K;
      _c[2] = _K*(_z*_dt - 1);
      break;
    case DigitialTransform::kBackwordDiff:
      n = 1./(1. + _p*_dt);
      _c[0] = n;
      _c[1] = _K*(1.0 + _z*_dt)*n;
      _c[2] = -_K*n;
      break;
  }
}

void LeadLag::init(const double &x0)
{
  _u     = 0.0;
  _error = 0.0;
}

double LeadLag::update(const double &x)
{
  double u_prev   = _u;
  double err_prev = _error;
  double err      = _target - x;

  _u = _c[0]*u_prev + _c[1]*err + _c[2]*err_prev;
  _error = err;
  return _u;
}
