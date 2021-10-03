

#include <libs/ode_solver/RungaKutta.hpp>
#include <libs/util/FunctionTimer.hpp>

#include <iostream>
#include <string>

using X_t = std::array<double, 2>;

/// Second-order ODE written as a system
/// of first order equations.
/// Where: x'' + 0.55x' + 30.25x = 0
X_t dx_dt(const X_t &x, const double u)
{
  return { x[1], -0.55*x[1] - 30.25*x[0] };
}

int main(int argc, char* arg[])
{
  std::function<X_t(const X_t&, const double)> func1 = dx_dt;
  RungaKutta<2> rk3(std::move(func1), RungaKutta<2>::SolverType::kThirdOrder);

  std::function<X_t(const X_t&, const double)> func2 = dx_dt;
  RungaKutta<2> rk4(std::move(func2), RungaKutta<2>::SolverType::kFourthOrderClassic);

  std::function<X_t(const X_t&, const double)> func3 = dx_dt;
  RungaKutta<2> rk_opt(std::move(func3), RungaKutta<2>::SolverType::kFourthOrderOptimal);

  const double y0    = 5.0;
  const double dt    = 0.08;
  const double t_max = 10.0;

  std::vector<double> t = {0.0};

  util::FunctionTimer tmr1;
  X_t x_rk3 = {y0, 0.0};
  std::vector<double> y_rk3 = {y0};

  util::FunctionTimer tmr2;
  X_t x_rk4 = {y0, 0.0};
  std::vector<double> y_rk4 = {y0};

  util::FunctionTimer tmr3;
  X_t x_rk_opt = {y0, 0.0};
  std::vector<double> y_rk_opt = {y0};

  while(true) {
    double t_next = *t.rbegin() + dt;
    if( t_next > t_max) { break; }
    t.push_back(t_next);

    tmr1.start();
    x_rk3 = rk3.step(x_rk3, 0.0, dt);
    tmr1.stop();
    y_rk3.push_back(x_rk3[0]);

    tmr2.start();
    x_rk4 = rk4.step(x_rk4, 0.0, dt);
    tmr2.stop();
    y_rk4.push_back(x_rk4[0]);

    tmr3.start();
    x_rk_opt = rk_opt.step(x_rk_opt, 0.0, dt);
    tmr3.stop();
    y_rk_opt.push_back(x_rk_opt[0]);
  }

  std::cout << "Final Results:\n";
  std::cout << "\t   RK3: " << std::to_string(*y_rk3.rbegin()) << std::endl;
  std::cout << "\t   RK4: " << std::to_string(*y_rk4.rbegin()) << std::endl;
  std::cout << "\tRK_OPT: " << std::to_string(*y_rk_opt.rbegin()) << std::endl;
  std::cout << "Timing:\n";
  std::cout << "\t   RK3: " << std::to_string(tmr1.average_us()) << std::endl;
  std::cout << "\t   RK4: " << std::to_string(tmr2.average_us()) << std::endl;
  std::cout << "\tRK_OPT: " << std::to_string(tmr3.average_us()) << std::endl;

}
