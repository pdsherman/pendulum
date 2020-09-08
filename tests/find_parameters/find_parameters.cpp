/*
  File:   model_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Physics based simulation for inverted pendulum
*/

#include <plant/SimpleModel.hpp>
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
    double weight = exp(0.08*xsim[i]);
    error += weight*pow(yinter-ysim[i], 2.0);
  }

  return error;
}

double run_trial(double b, double I, const std::vector<double> &t, const std::vector<double> &theta)
{
  double theta0 = theta[0];
  std::vector<double> tsim{0.0};
  std::vector<double> thetasim{theta0};

  SimpleModel pendulum(theta0, b, I);

  const double dt = 0.01; // Time step
  double testtime = 0.0;
  while(testtime < 60.0) {
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
  std::string infile = "encoder_data.csv";

  auto data = get_data_from_csv(directory + infile);
  std::vector<double> t(std::move(data["test_time_s"]));
  std::vector<double> theta(std::move(data["theta"]));

  double z = t[0];
  std::for_each(t.begin(), t.end(), [z](double &i){ i -= z; });

  double b = 0.0;
  double I = 0.015;

  double b_eps = 0.000005;
  double I_eps = 0.00002;

  double err_min = std::numeric_limits<double>::max();
  double b_min = std::numeric_limits<double>::max();
  double I_min = std::numeric_limits<double>::max();

  for(size_t Ii = 0; Ii < 500; ++Ii) {
    b = 0.0;
    for(size_t bi = 0; bi < 1000; ++bi){
      double err = run_trial(b, I, t, theta);

      if(err < err_min) {
        b_min = b;
        I_min = I;
        err_min = err;
      }

      b += b_eps;
    }
    I += I_eps;
  }

  std::cout << "b_min: " << b_min << std::endl;
  std::cout << "I_min: " << I_min << std::endl;

  double theta0 = theta[0];
  std::vector<double> tsim{0.0};
  std::vector<double> thetasim{theta0};

  SimpleModel pendulum(theta0, b_min, I_min);

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
