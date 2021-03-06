
#include <plant/Model.hpp>

#include <libs/util/util.hpp>

#include <cmath>


std::shared_ptr<Plant<4>> Model::create(const State &x0)
{
  return static_cast<std::shared_ptr<Plant<4>>>(std::make_shared<Model>(x0));
}

Model::Model(const State &x0) : Plant<4>(x0)
{
}

Model::State Model::update(const double u, const double dt)
{
  /// Method will calculate the state of the system one
  /// time step in the future by using classical 4th order
  /// Runga-Kutta numerical sovler.

  std::array<std::array<double, 4>, 4> kx;

  for(int ii = 0; ii < 4; ++ii) {
    std::array<double, 4> delta;
    for(size_t jj = 0; jj < 4; ++jj) {
      switch(ii){
        case 0:
          delta[jj] = 0.0;
          break;
        case 1:
          delta[jj] = kx[jj][0]/2.0;
          break;
        case 2:
          delta[jj] = kx[jj][1]/2.0;
          break;
        case 3:
          delta[jj] = kx[jj][2];
          break;
      }
    }

    State x_dot = calculate_x_dot(util::add_arrays<double, 4>(_x, delta), u);
    for(size_t jj = 0; jj < 4; ++jj)
      kx[jj][ii] = dt*x_dot[jj];
  }

  std::array<double, 4> dx;
  for(size_t jj = 0; jj < 4; ++jj)
    dx[jj] = (kx[jj][0] + 2*kx[jj][1] + 2*kx[jj][2] + kx[jj][3])/6.0;

  _x = util::add_arrays<double, 4>(_x, dx);

  return _x;
}

Model::State Model::calculate_x_dot(const State &x, const double u) const
{
  static constexpr double d = M + m;
  static constexpr double k = m*l;
  static constexpr double w = b_p/l;
  static constexpr double z = (m*l) + (I/l);

  const double c2 = cos(x[2]);
  const double s2 = sin(x[2]);
  const double s2_sqr = pow(s2, 2.0);

  State x_dot;

  // x-dot
  x_dot[0] = x[1];

  // x-double-dot
  const double n1 = z / (d*z - m*k*s2_sqr);
  const double n2 = (2.0*k + z) / z;
  x_dot[1] = n1*(-b_b*x[1] + k*c2*pow(x[3], 2.0) - n2*w*s2*x[3] - m*k*g*sin(2*x[2])/(2*z) + u);

  // theta-dot
  x_dot[2] = x[3];

  // theta-double-dot
  const double n3 = m / (d*z - m*k*s2_sqr);
  const double n4 = 0.5*k*sin(2.0*x[2]);
  const double n5 = (2.0*d + m*s2_sqr) / m;
  x_dot[3] = n3*(-b_b*s2*x[1] + n4*pow(x[3], 2.0) - n5*w*x[3] - d*g*c2 + s2*u);

  return x_dot;
}
