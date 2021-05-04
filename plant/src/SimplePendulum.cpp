
#include <plant/SimplePendulum.hpp>

#include <libs/util/util.hpp>

#include <cmath>

std::shared_ptr<Plant<4>> SimplePendulum::create(const X_t &x0)
{
  return static_cast<std::shared_ptr<Plant<4>>>(std::make_shared<SimplePendulum>(x0));
}

SimplePendulum::SimplePendulum(const X_t &x0) :
  Plant<4>(x0,
    [&](const X_t& x, const double u){ return this->calculate_x_dot(x, u); }),
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

SimplePendulum::X_t SimplePendulum::calculate_x_dot(const X_t &x, const double u) const
{
  X_t x_dot;

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
