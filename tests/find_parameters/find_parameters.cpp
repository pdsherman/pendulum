/*
  File:   model_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Physics based simulation for inverted pendulum
*/

#include <plant/PlantManager.hpp>
#include <plant/SimplePendulum.hpp>

#include <libs/util/util.hpp>

#include <pendulum/State.h>
#include <pendulum/RemovePendulum.h>
#include <pendulum/GraphData.h>
#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingDropTable.h>
#include <pendulum/LoggingBufferCheck.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <algorithm>
#include <cmath>
#include <future>
#include <thread>
#include <limits>
#include <fstream>

double run_trial(
  ros::NodeHandle &nh,
  const std::string &name,
  const double b,
  const double I,
  const std::vector<double> &t,
  const std::vector<double> &theta);

double calc_error(
  const std::vector<double> &xtarget, const std::vector<double> &ytarget,
  const std::vector<double> &xsim, const std::vector<double> &ysim);

bool start_logging(
  ros::NodeHandle &nh,
  const std::string &name,
  const std::string &log_table_name);

void logging_wait_until_done(ros::NodeHandle &nh);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Simulation");
  ros::NodeHandle nh;

  const std::string directory = "/home/pdsherman/projects/pendulum/catkin_ws/src/pendulum/data/";
  const std::string infile = "encoder_data.csv";
  const std::string name = "Simulated";

  auto sensor_data = util::read_data_from_csv(directory + infile);
  std::vector<double> t(std::move(sensor_data["test_time_s"]));
  std::vector<double> theta(std::move(sensor_data["theta"]));
  for(auto &x : t) { x -= t[0]; }



  double b_best = 0.0;
  double b_min = 0.00050;
  double b_max = 0.00070;
  double b_eps = 0.00002;

  double I_best = 0.0; // [0.00415, 0.00508] 0.00461 is original
  double I_max = 0.00477;
  double I_min = 0.00467;
  double I_eps = 0.00001;

  int c = 0;
  double err_min = std::numeric_limits<double>::max();
  for(double I = I_min; I <= I_max; I += I_eps) {
    for(double b = b_min; b <= b_max; b += b_eps) {
      double err = run_trial(nh, name, b, I, t, theta);
      if(err < err_min) {
        err_min = err;
        b_best = b;
        I_best = I;
      }
    }
  }

  I_best = 0.00474;
  b_best = 0.00061;

  ROS_INFO("b: %f", b_best);
  ROS_INFO("I: %f", I_best);


  std::string table_name = "SimpleSim" + std::to_string(c++);

  if(start_logging(nh, name, table_name)) {
    ros::Duration(0.1).sleep();
    run_trial(nh, name, b_best, I_best, t, theta);
    logging_wait_until_done(nh);
  }

  ros::ServiceClient plot_client = nh.serviceClient<pendulum::GraphData>("/grapher/plot_data");
  if(plot_client.exists()) {
    pendulum::GraphData grapher;
    grapher.request.table_name = table_name;
    grapher.request.y_axis_data = "Y";
    grapher.request.x_axis_data = "X";
    grapher.request.use_time_bound = false;
    grapher.request.overlay = true;

    plot_client.call(grapher);
  }

  return 0;
}


double run_trial(
  ros::NodeHandle &nh,
  const std::string &name,
  const double b,
  const double I,
  const std::vector<double> &t,
  const std::vector<double> &theta)
{
  const double u0 = 0.0;  // Initial u0
  const double dt = 0.02; // Time step

  double theta0 = theta[0];
  std::vector<double> tsim{0.0};
  std::vector<double> thetasim{theta0};

  pendulum::State x0;
  x0.x           = 1.0;
  x0.x_dot       = 0.0;
  x0.theta       = theta[0];
  x0.theta_dot   = 0.0;
  x0.test_time_s = 0.0;

  PlantManager pendulum(nh, name, x0, u0, dt, PlantManager::Type::Simple);
  std::shared_ptr<SimplePendulum> plant = std::dynamic_pointer_cast<SimplePendulum>(pendulum.get_plant());
  plant->set_moment_of_inertia(I);
  plant->set_friction(b);
  ros::Duration(0.3).sleep();

  ros::Time test_time = ros::Time::now();
  ros::Time end_time  = test_time + ros::Duration(60.0);
  pendulum.initialize_test(test_time);

  while(ros::ok() && test_time < end_time) {
    test_time += ros::Duration(dt);

    // First get all updates
    ros::spinOnce();

    // Simulate system one time step forward.
    pendulum.cycle(test_time);

    // Get simulation state
    tsim.push_back(pendulum.get_current_state().test_time_s);
    thetasim.push_back(pendulum.get_current_state().theta);

    //ros::Duration(0.001).sleep();
  }

  return calc_error(t, theta, tsim, thetasim);
}

void logging_wait_until_done(ros::NodeHandle &nh)
{
  ros::Time end = ros::Time::now() + ros::Duration(120.0);

  ros::ServiceClient client_logging_done = nh.serviceClient<pendulum::LoggingBufferCheck>("/sqlite/log_buffer_check");
  if(client_logging_done.exists()) {
    while(ros::Time::now() <= end) {
      pendulum::LoggingBufferCheck logging_done;
      client_logging_done.call(logging_done);
      if(logging_done.response.is_empty && logging_done.response.size == 0) {
        ROS_INFO("Logging Completed.");
        return;
      }
      ros::Duration(0.2).sleep();
    }
    ROS_ERROR("Timeout waiting for logging to complete.");
  } else {
    ROS_ERROR("Logging buffer check client doesn't exist.");
  }
}

double calc_error(
  const std::vector<double> &xtarget, const std::vector<double> &ytarget,
  const std::vector<double> &xsim, const std::vector<double> &ysim)
{
  double error = 0.0;
  size_t length = std::min(xsim.size(), xtarget.size());

  for(size_t i = 0; i < length; ++i) {
    double yinter = util::interpolate(xtarget, ytarget, xsim[i]);
    double weight = exp(0.023*xsim[i]);
    error += weight*pow(yinter-ysim[i], 2.0);
  }

  return error;
}

bool start_logging(ros::NodeHandle &nh, const std::string &name, const std::string &log_table_name)
{
  // Delete Table if exists to avoid data collision
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingDropTable>("/sqlite/drop_table");
  if(sql_client.exists()) {
    pendulum::LoggingDropTable drop_table;
    drop_table.request.table_name = log_table_name;

    sql_client.call(drop_table);
    if(!drop_table.response.success)
      ROS_WARN("Failure to drop table");
  }

  // Request logging node to start logging
  sql_client = nh.serviceClient<pendulum::LoggingStart>("/sqlite/start_log");
  if(sql_client.exists()) {
    pendulum::LoggingStart start_log;
    start_log.request.table_name = log_table_name;
    start_log.request.topic_name = name;

    return sql_client.call(start_log) && start_log.response.success;
  }
  return false;
}
