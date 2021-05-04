
#include <plant/PendulumModel.hpp>

#include <libs/util/util.hpp>


#include <cmath>


std::shared_ptr<Plant<4>> PendulumModel::create(const X_t &x0)
{
  return static_cast<std::shared_ptr<Plant<4>>>(std::make_shared<PendulumModel>(x0));
}

PendulumModel::PendulumModel(const X_t &x0) :
  Plant<4>(x0,
    [&](const X_t& x, const double u){ return this->calculate_x_dot(x, u); })
{
}

PendulumModel::X_t PendulumModel::calculate_x_dot(const X_t &x, const double u) const
{
  static constexpr double d = M + m;
  static constexpr double k = m*l;
  static constexpr double w = b_p/l;
  static constexpr double z = (m*l) + (I/l);

  const double c2 = cos(x[2]);
  const double s2 = sin(x[2]);
  const double s2_sqr = pow(s2, 2.0);

  X_t x_dot;

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
