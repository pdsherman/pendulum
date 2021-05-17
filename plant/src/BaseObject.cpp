
#include <plant/BaseObject.hpp>
#include <libs/util/util.hpp>

#include <cmath>


std::shared_ptr<Plant<2>> BaseObject::create(const X_t &x0)
{
  return static_cast<std::shared_ptr<Plant<2>>>(std::make_shared<BaseObject>(x0));
}

BaseObject::BaseObject(const X_t &x0) : Plant<2>(x0,
  [&](const X_t& x, const double u){ return this->calculate_x_dot(x, u); })
{
}

BaseObject::X_t BaseObject::calculate_x_dot(const X_t &x, const double u) const
{
  X_t x_dot;
  x_dot[0] = x[1];
  x_dot[1] = (1/M)*(-b_b * x[1] + u);
  return x_dot;
}
