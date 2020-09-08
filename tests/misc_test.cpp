

#include <array>
#include <cmath>
#include <iostream>

static constexpr double M = 5.5;
static constexpr double m = 2.7;
static constexpr double l = 0.5;
static constexpr double I = 0.188;
static constexpr double b_b = 200.5;
static constexpr double b_p = 0.08;

static constexpr double g = 9.80665;

std::array<double, 4> calculate_x_dot(const std::array<double, 4> &x, const double u)
{
  static constexpr double d = M + m;
  static constexpr double k = m*l;
  static constexpr double w = b_p/l;
  static constexpr double z = (m*l) + (I/l);

  const double c2 = cos(x[2]);
  const double s2 = sin(x[2]);

  std::array<double, 4> x_dot;

  // x-dot
  x_dot[0] = x[1];

  // x-double-dot
  double n1 = z/(d*z - m*k*pow(s2, 2.0));
  double n2 = w*s2 - 2*k*w*s2/z;
  x_dot[1] = n1*(-b_b*x[1] + n2*x[3] + k*c2*pow(x[3], 2.0) - k*m*g*c2*c2/z + u);

  // theta-dot
  x_dot[2] = x[3];

  // theta-double-dot
  double n3 = (m*s2)/(d*z - m*k*pow(s2, 2.));
  double n4 = w*s2 - (2*d*w)/(m*s2);
  x_dot[3] = n3*(n4*x[3] + k*c2*pow(x[3], 2.) - b_b*x[1] - g*d*c2/s2 + u);

  return x_dot;
}


int main(int argc, char *argv[])
{
  std::array<double, 4> x0;
  x0[0] = 0.0;
  x0[1] = 0.0;
  x0[2] = 0.0;
  x0[3] = 0.0;

  double u = 0;

  std::array<double, 4> x_dot = calculate_x_dot(x0, u);
  std::cout << "x_dot: "      << std::to_string(x_dot[0]) << std::endl;
  std::cout << "x_ddot: "     << std::to_string(x_dot[1]) << std::endl;
  std::cout << "theta_dot: "  << std::to_string(x_dot[2]) << std::endl;
  std::cout << "theta_ddot: " << std::to_string(x_dot[3]) << std::endl;

  return 0;
}
