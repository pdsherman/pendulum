
#include <plant/SimplePendulum.hpp>

#include <libs/util/util.hpp>

#include <cmath>

std::shared_ptr<Plant<4>> SimplePendulum::create(const State &x0)
{
  return static_cast<std::shared_ptr<Plant<4>>>(std::make_shared<SimplePendulum>(x0));
}

SimplePendulum::SimplePendulum(const State &x0) :
  Plant<4>(x0),
  _b(1.0),
  _I(1.0)
{
}

void SimplePendulum::set_moment_of_inertia(const double I)
{
  _I = I;
}

void SimplePendulum::set_friction(const double b)
{
  _b = b;
}

SimplePendulum::State SimplePendulum::update(const double u, const double dt)
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

SimplePendulum::State SimplePendulum::calculate_x_dot(const State &x, const double u) const
{
  State x_dot;

  // Ignoring linear motion
  x_dot[0] = 0.0;
  x_dot[1] = 0.0;

  // Theta-dot
  x_dot[2] = x[3];

  // Theta-double-dot
  const double c = l/(m*l*l + _I);
  x_dot[3] = c*((-2.0*_b/l)*x[3] - m*g*cos(x[2]));

  return x_dot;
}
