/*
  @file:    control_base.cpp
  @author:  pdsherman
  @date:    April. 2022
*/

#include <libs/ode_solver/RungaKutta.hpp>
#include <libs/util/log_util.hpp>

#include <iostream>
#include <cmath>
#include <utility>
#include <functional>

// x = [theta, theta_dot]
using X_t = std::array<double, 2>;

static constexpr double m = 0.245; //< mass of pendulum (kilogram)
static constexpr double l = 0.251; //< length to midpoint (meters)
static constexpr double g = 9.80665;  //< Acceleration from gravity (m/s^2)
static constexpr double mgl = m*g*l;

static constexpr double I   = 0.0221;
static constexpr double I_r = 1/I;

static double b = 0.00; // later on, this will vary


std::map<std::string, std::vector<double>> get_data(void);
double sgn(double val);
std::vector<std::vector<double>> simulation(std::function<X_t(const X_t&, const double)> func,
                            const X_t &x0, const double time_total,const double dt);


// x'' = 1/I*(-b*x' - m*g*l*cos(x))
X_t friction(const X_t &x, const double u)
{
  X_t dx_dt;
  dx_dt[0] = x[1];
  dx_dt[1] = I_r * (-b*x[1] - mgl*cos(x[0]));
  return dx_dt;
}

// x'' = 1/I*(-b*x'^2*sgn(x') - m*g*l*cos(x))
X_t drag(const X_t &x, const double u)
{
  X_t dx_dt;
  dx_dt[0] = x[1];
  dx_dt[1] = I_r * (-b*x[1]*x[1]*sgn(x[1]) - mgl*cos(x[0]));
  return dx_dt;
}

int main(int argc, char *argv[])
{
  std::string suffix = "";
  if(argc > 1) {
    try {
      b = std::stod(argv[1]);
      int pos = std::string(argv[1]).find('.');
      suffix = "_" + std::string(argv[1]).substr(pos+1);
    } catch (const std::invalid_argument& ia) {
      std::cerr << "Invalid input for b: " << ia.what() << std::endl;
      b = 0.0;
    }
  }

  auto data = get_data();

  X_t x0 = {data["theta"][0], 0.0};
  double total_time = data["test_time"].back();
  double dt         = 0.02;

  std::function<X_t(const X_t&, const double)> func1 = friction;
  auto friction_sim = simulation(friction, x0, total_time, dt);
  if(!util::write_data_to_db("SimplePendulumSim", "friction"+suffix,
    {"model","test_time","theta","theta_dot"}, friction_sim))
    std::cout << "Failed to write to database" << std::endl;

  std::function<X_t(const X_t&, const double)> func2 = drag;
  auto drag_sim = simulation(drag, x0, total_time, dt);
  if(!util::write_data_to_db("SimplePendulumSim", "drag"+suffix,
    {"model","test_time","theta","theta_dot"}, drag_sim))
    std::cout << "Failed to write to database" << std::endl;

  return 0;
}

std::vector<std::vector<double>> simulation(std::function<X_t(const X_t&, const double)> func,
                            const X_t &x0, const double time_total, const double dt)
{
  RungaKutta<2> rk(std::move(func), RungaKutta<2>::SolverType::kFourthOrderOptimal);
  std::vector<std::vector<double>> sim; // [t, x(t), x'(t)]

  double t = 0.0;
  sim.push_back({t, x0[0], x0[1]});
  while(t < time_total) {
    X_t x  = {sim.back()[1], sim.back()[2]};
    X_t x1 = rk.step(x, 0.0, dt);
    t += dt;
    sim.push_back({t, x1[0], x1[1]});
  }
  return sim;
}


std::map<std::string, std::vector<double>> get_data(void)
{
  std::string table_name = "EncoderOneTest";
  std::string test_name  = "real_03";

  std::string cmd = "SELECT * FROM " + table_name +
  " WHERE test_name = '" + test_name + "' ORDER BY test_time";

  auto data = util::read_data_from_db(cmd);

  // Look for point when theta starts dropping
  int idx = 0;
  for(size_t i = 0; i < data["theta"].size()-1; ++i) {
    if(data["theta"][i+1] < data["theta"][i]) {
      idx = i;
      break;
    }
  }
  data["theta"].erase(data["theta"].begin(), data["theta"].begin()+idx);
  data["test_time"].erase(data["test_time"].begin(), data["test_time"].begin()+idx);

  // Update the times to start at zero
  double t0 = data["test_time"][0];
  for(auto &t : data["test_time"]) { t = t-t0; }

  return data;
}

double sgn(double val) {
    return (0.0 < val) - (val < 0.0);
}
