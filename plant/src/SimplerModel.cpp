
#include <plant/SimplerModel.hpp>

#include <array>

SimplerModel::SimplerModel(double theta0, double b, double I) :
  _theta(theta0), _velocity(0.0),
  _b(b),
  _I(I)
{
}

double SimplerModel::get_current_state(void) const
{
  return _theta;
}

void SimplerModel::update(const double h)
{
  /// Method will calculate the state of the system one
  /// time step in the future by using classical 4th order
  /// Runga-Kutta numerical sovler.

  std::array<double, 4> kt, ft;

  for(int ii = 0; ii < 4; ++ii) {
    double dt, ddt;
    switch(ii){
      case 0:
        dt  = 0.0;
        ddt = 0.0;
      case 1:
        dt  = kt[0]/2.0;
        ddt = ft[0]/2.0;
        break;
      case 2:
        dt  = kt[1]/2.0;
        ddt = ft[1]/2.0;
        break;
      case 3:
        dt  = kt[2];
        ddt = ft[2];
        break;
    }

    double accel = calculate_accel(_theta + dt, _velocity + ddt);
    ft[ii] = h*accel;
    kt[ii] = h*(_velocity + ddt);
  }

  // Combine and update states
  double dtheta = (kt[0] + 2*kt[1] + 2*kt[2] + kt[3])/6.0;
  double dtheta_dot = (ft[0] + 2*ft[1] + 2*ft[2] + ft[3])/6.0;

  _theta    += dtheta;
  _velocity += dtheta_dot;
}

double SimplerModel::calculate_accel(const double theta, const double velocity) const
{
    // Calculate rotational acceleration of theta
    double n1  = _b/_I;
    double n2 = k/_I;
    double accel = -n1*velocity - n2*theta;

    return accel;
}
