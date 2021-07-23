
#include <plant/ExampleDigital.hpp>
#include <libs/util/util.hpp>

#include <cmath>


std::shared_ptr<Plant<2>> ExampleDigital::create(const X_t &x0)
{
  return static_cast<std::shared_ptr<Plant<2>>>(std::make_shared<ExampleDigital>(x0));
}

ExampleDigital::ExampleDigital(const X_t &x0) : Plant<2>(x0,
  [this](const X_t& x, const double u){ return this->calculate_x_dot(x, u); })
{
}

ExampleDigital::X_t ExampleDigital::calculate_x_dot(const X_t &x, const double u) const
{
  X_t x_dot;
  x_dot[0] = x[1];
  x_dot[1] = -36000.0*x[0] - 660.0*x[1] + 360000*u;
  return x_dot;
}
