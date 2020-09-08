/*
  File:   model_publisher.cpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Physics based simulation for inverted pendulum
*/

#include <plant/PlantManager.hpp>
#include <plant/SimplePendulum.hpp>

#include <pendulum/State.h>
#include <pendulum/AddPendulum.h>
#include <pendulum/RemovePendulum.h>
#include <pendulum/GraphData.h>
#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingDropTable.h>
#include <pendulum/LoggingBufferEmpty.h>

#include <ros/ros.h>
#include <ros/console.h>

#include <atomic>
#include <csignal>
#include <memory>


/// Request GUI node to display the pendulum
static bool display(ros::NodeHandle &nh, const std::string &name, const pendulum::State &x0);

/// Call logging node to subscribe to topic and being logging data
static bool start_logging(ros::NodeHandle &nh, const std::string &name, const std::string &log_table_name);

/// Plot results logged in table
static void plot_test_data(ros::NodeHandle &nh, const std::string &log_table_name);

/// Catch user exit signal and allow for ending actions
static std::atomic<bool> sigint_raised{false};

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "Simulation");
  ros::NodeHandle nh;

  std::signal(SIGINT, [](int) { sigint_raised = true; });
  bool logging_active = false;
  std::string log_table_name = "ModelPublisher01";

  // Initial conditions
  const double u0 = 0.0;

  pendulum::State x0;
  x0.x           = 1.0;
  x0.x_dot       = 0.0;
  x0.theta       = 0.5183;
  x0.theta_dot   = 0.0;
  x0.test_time_s = 0.0;

  std::string name = "Simulated";

  // Activate logging to database
  logging_active = start_logging(nh, name, log_table_name);
  ros::Duration(0.1).sleep();

  // Attempt to Add pendulum to gui. Exit if servicer server doesn't start up in time.
  if(!display(nh, name, x0)) {
    ROS_WARN("Unable to display. Ending application.");
    return 1;
  }

  // Run simulation & publish state data
  const double dt = 0.01; // Time step
  ros::Rate rate(1/dt);

  PlantManager::Type plant_type = PlantManager::Type::Simple;
  PlantManager pendulum(nh, name, x0, u0, dt, plant_type);

  if(plant_type ==  PlantManager::Type::Simple) {
    std::shared_ptr<SimplePendulum> plant = std::dynamic_pointer_cast<SimplePendulum>(pendulum.get_plant());
    plant->set_moment_of_inertia(0.02);
    plant->set_friction(0.004);
  }

  ros::Duration(0.5).sleep();
  pendulum.initialize_test(ros::Time::now());

  // ************************** //
  // **      MAIN LOOP       ** //
  // ************************** //
  ROS_INFO("Beginning simulation.");
  while(ros::ok() && !sigint_raised)
  {
    // First get all updates
    ros::spinOnce();

    // Simulate system one time step forward.
    pendulum.cycle(ros::Time::now());

    // Sleep until next cycle
    rate.sleep();
  }
  ROS_INFO("Ending simulation.");

  // Waits for all data to be inserted into SQLite table
  ros::ServiceClient client_logging_done = nh.serviceClient<pendulum::LoggingBufferEmpty>("/grapher/log_buffer_empty");
  if(client_logging_done.exists()) {
    while(true) {
      pendulum::LoggingBufferEmpty logging_done;
      client_logging_done.call(logging_done);
      ros::Duration(0.2).sleep();
      if(logging_done.response.is_empty) { break; }
    }
  }

  // Remove pendulum object from GUI
  ros::ServiceClient client_removal = nh.serviceClient<pendulum::RemovePendulum>("/gui/remove_pendulum");
  if(client_removal.exists()) {
    pendulum::RemovePendulum removal_srv;
    removal_srv.request.name  = name;
    client_removal.call(removal_srv);
  }

  // Plot results
  if(logging_active) { plot_test_data(nh, log_table_name); }

  return 0;
}

bool display(ros::NodeHandle &nh, const std::string &name, const pendulum::State &x0)
{
  ros::ServiceClient client = nh.serviceClient<pendulum::AddPendulum>("/gui/add_pendulum");

  int count = 0;
  while(!client.exists() && count < 10) {
    ros::Rate(2).sleep();
    ++count;
  }

  if(count >= 10) {
    ROS_WARN("Timeout waiting for AddPendulum service to exist.");
    return false;
  }

  pendulum::AddPendulum add_srv;
  add_srv.request.name  = name;
  add_srv.request.x     = x0.x;
  add_srv.request.theta = x0.theta;
  add_srv.request.base_color = "blue";
  add_srv.request.pendulum_color = "#FC33FF";

  return client.call(add_srv);
}

bool start_logging(ros::NodeHandle &nh, const std::string &name, const std::string &log_table_name)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingDropTable>("/sqlite/drop_table");
  if(sql_client.exists()) {
    pendulum::LoggingDropTable drop_table;
    drop_table.request.table_name = log_table_name;

    sql_client.call(drop_table);
    if(!drop_table.response.success)
      ROS_WARN("Failure to drop table");
  }

  sql_client = nh.serviceClient<pendulum::LoggingStart>("/sqlite/start_log");
  if(sql_client.exists()) {
    pendulum::LoggingStart start_log;
    start_log.request.table_name = log_table_name;
    start_log.request.topic_name = name;
    start_log.request.start_time = ros::Time::now();

    return sql_client.call(start_log) && start_log.response.success;
  }
  return false;
}

void plot_test_data(ros::NodeHandle &nh, const std::string &log_table_name)
{
    ros::ServiceClient plot_client = nh.serviceClient<pendulum::GraphData>("/grapher/plot_data");
    if(plot_client.exists()) {
      pendulum::GraphData grapher;
      grapher.request.table_name = log_table_name;
      grapher.request.y_axis_data = "Y";
      grapher.request.x_axis_data = "X";
      grapher.request.use_time_bound = false;
      grapher.request.overlay = true;

      plot_client.call(grapher);
    }
}
