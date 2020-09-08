/*
  File:   model_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Physics based simulation for inverted pendulum
*/

#include <plant/SimpleModel.hpp>
#include <libs/util/util.hpp>

#include <ros/ros.h>
#include <ros/console.h>

#include <algorithm>
#include <chrono>
#include <iostream>
#include <fstream>
#include <cmath>

double calc_error(
  const std::vector<double> &xtarget, const std::vector<double> &ytarget,
  const std::vector<double> &xsim, const std::vector<double> &ysim)
{
  double error = 0.0;

  for(size_t i = 0; i < xsim.size(); ++i) {
    double yinter = interpolate(xtarget, ytarget, xsim[i]);
    error += pow(yinter-ysim[i], 2.0);
  }

  return error;
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "simulation");
  ros::NodeHandle nh;

  const std::string directory = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/";
  std::string infile = "encoder_data.csv";

  auto data = get_data_from_csv(directory + infile);
  std::vector<double> t(std::move(data["test_time_s"]));
  std::vector<double> theta(std::move(data["theta"]));

  double z = t[0];
  std::for_each(t.begin(), t.end(), [z](double &i){ i -= z; });

  // Initial conditions
  double theta0 = 0.50265;
  std::string name = "simulation";
  const double dt = 0.01; // Time step

  std::string error = "errorfile.csv";
  std::ofstream err_fs;
  err_fs.open(directory + error, std::ios::out | std::ios::trunc);
  err_fs << "b,I,err\n";

  double b_min = 0.0;
  double I_min = 0.0;
  double err_min = -1.0;

  double b = 0.0012;
  double I = 0.01;
  //for(double b = 0.0005; b <= 0.0020; b += 0.0001) {
  //  for(double I = 0.01; I <= 0.03; I += 0.002){
        SimpleModel pendulum(nh, name, theta0, 0.0012, 0.01);

        std::vector<double> tsim{0.0};
        std::vector<double> thetasim{theta0};

        double testtime = 0.0;
        while(testtime < 62.4) {
          testtime += dt;
          pendulum.update(dt);

          tsim.push_back(testtime);
          thetasim.push_back(pendulum.get_current_state());
        }

        double err = calc_error(t, theta, tsim, thetasim);
        err_fs << b << "," << I << "," << err << "\n";

        if(b_min == 0.0 && I_min == 0.0) {
          b_min = b;
          I_min = I;
          err_min = err;
        } else if(err < err_min) {
          b_min = b;
          I_min = I;
          err_min = err;
        }


        std::string outfile = "simulated.csv";
        std::ofstream ofs;
        ofs.open(directory + outfile, std::ios::out | std::ios::trunc);
        ofs << "test_time_s,theta\n";
        for(size_t i = 0; i < tsim.size() ; ++i)
          ofs << tsim[i] << "," << thetasim[i] << "\n";
        ofs.close();

//    }
//  }

  std::cout << "b: " << b_min << std::endl;
  std::cout << "I: " << I_min << std::endl;
  std::cout << "Err: " << err_min << std::endl;

  err_fs.close();

  return 0;
}
