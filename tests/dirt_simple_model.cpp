/*
  File:   model_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Physics based simulation for inverted pendulum
*/

#include <plant/SimplerModel.hpp>
#include <libs/util/util.hpp>

#include <algorithm>
#include <iostream>
#include <cmath>
#include <future>
#include <thread>
#include <limits>
#include <fstream>

double calc_error(
  const std::vector<double> &xtarget, const std::vector<double> &ytarget,
  const std::vector<double> &xsim, const std::vector<double> &ysim)
{
  double error = 0.0;
  size_t length = std::min(xsim.size(), xtarget.size());

  for(size_t i = 0; i < length; ++i) {
    double yinter = interpolate(xtarget, ytarget, xsim[i]);
    double weight = exp(0.035*xsim[i]);
    error += weight*pow(yinter-ysim[i], 2.0);
  }

  return error;
}

double run_trial(double b, double I, const std::vector<double> &t, const std::vector<double> &theta)
{
  static constexpr double theta0 = 1.0;
  std::vector<double> tsim{0.0};
  std::vector<double> thetasim{theta0};

  SimplerModel pendulum(theta0, b, I);

  const double dt = 0.01; // Time step
  double testtime = 0.0;
  while(testtime < 62.4) {
    testtime += dt;
    pendulum.update(dt);

    tsim.push_back(testtime);
    thetasim.push_back(pendulum.get_current_state());
  }
  return calc_error(t, theta, tsim, thetasim);
}

int main(int argc, char *argv[])
{
  const std::string directory = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/";
  std::string infile = "fake_sensor.csv";

  auto data = get_data_from_csv(directory + infile);
  std::vector<double> t(std::move(data["test_time_s"]));
  std::vector<double> theta(std::move(data["theta"]));

  double b = 0.021; // Final target: 0.013
  double I = 0.15;  // Final target: 0.18

  double eps = 0.0005;

  double err_min = std::numeric_limits<double>::max();
  double b_min = std::numeric_limits<double>::max();
  double I_min = std::numeric_limits<double>::max();

  for(size_t i = 0; i < 50000; ++i){
    std::future<double> f0 = std::async(std::launch::async, run_trial, b, I, t, theta);
    std::future<double> f1 = std::async(std::launch::async, run_trial, b+eps, I, t, theta);
    double err_2 = run_trial(b, I+eps, t, theta);

    double err_0 = f0.get();
    double err_1 = f1.get();

    b -= (err_1-err_0) * eps;
    I -= (err_2-err_0) * eps;

    if(err_0 < err_min) {
      I_min = I;
      b_min = b;
      err_min = err_0;
    }
  }

  std::cout << "b: " << b << std::endl;
  std::cout << "I: " << I << std::endl;
  std::cout << "b_min: " << b_min << std::endl;
  std::cout << "I_min: " << I_min << std::endl;

  double theta0 = 1.0;
  std::vector<double> tsim{0.0};
  std::vector<double> thetasim{theta0};

  SimplerModel pendulum(theta0, b_min, I_min);

  const double dt = 0.01; // Time step
  double testtime = 0.0;
  while(testtime < 62.4) {
    testtime += dt;
    pendulum.update(dt);

    tsim.push_back(testtime);
    thetasim.push_back(pendulum.get_current_state());
  }
  double err = calc_error(t, theta, tsim, thetasim);

  std::cout << "Err: " << err << std::endl;

  std::string outfile = "simulated.csv";
  std::ofstream ofs;
  ofs.open(directory + outfile, std::ios::out | std::ios::trunc);
  ofs << "test_time_s,theta\n";
  for(size_t i = 0; i < tsim.size() ; ++i)
    ofs << tsim[i] << "," << thetasim[i] << "\n";
  ofs.close();


  return 0;
}
