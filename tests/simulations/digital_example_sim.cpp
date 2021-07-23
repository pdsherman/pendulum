/*
  @file:    control_base.cpp
  @author:  pdsherman
  @date:    April. 2021

  @brief: control base object
*/

#include <pendulum/LoggingData.h>
#include <pendulum/State.h>
#include <pendulum/Control.h>

#include <plant/ExampleDigital.hpp>
#include <libs/control/Pid.hpp>
#include <libs/control/DigitalPid.hpp>
#include <libs/control/LeadLag.hpp>
#include <libs/util/ros_util.hpp>

#include <ros/ros.h>
#include <iostream>
#include <thread>   // For this_thread::sleep_until
#include <chrono>

static constexpr double kRunTime_s   = 0.010;  // 10.0 msec
static constexpr double step_digital = 0.0003; //  0.3 msec
static constexpr double step_contin  = 0.00001; //  0.3 msec


static constexpr double Kp = 5.0;
static constexpr double Ki = 1666.67;
static constexpr double Kd = 0.004;

void publish(ros::Publisher &log_pub,
  pendulum::LoggingData &data, pendulum::State &state,
  const double target, const double u, const ExampleDigital::X_t &x, const double t);

double get_target(const double test_time);

/// ---------------------------------------- ///
/// ----        MAIN FUNCTION           ---- ///
/// ---------------------------------------- ///
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "digital_example_control");
  ros::NodeHandle nh;

  // ---   Start Logging   --- //
  std::string logging_topic = "digital_example";
  ros::Publisher log_pub = nh.advertise<pendulum::LoggingData>("/" + logging_topic, 50);
  std::string log_table = "DigitalExampleTest";
  std::vector<std::string> header({
    "test", "test_time", "target", "input", "x", "x_dot"
  });

  if(!util::drop_logging_table(nh, log_table) ||
     !util::start_logging(nh, log_table, logging_topic, header)) {
    ROS_WARN("Failed to start logging.");
  }

  // --- Test Parameters   --- //
  std::vector<std::pair<
     std::shared_ptr<Controller<double, double>>,
     std::string>> parameters;
  std::shared_ptr<Controller<double, double>> cntrl;

  std::map<std::string, double> gains({
    {"Kp", Kp},
    {"Ki", Ki},
    {"Kd", Kd}});

  cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
    std::make_shared<PID>(step_contin, gains["Kp"], gains["Ki"], gains["Kd"], 0.0));
  parameters.push_back({cntrl, "Continuous"});

  cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
    std::make_shared<PID>(step_digital, gains["Kp"], gains["Ki"], gains["Kd"], 0.0));
  parameters.push_back({cntrl, "Digital-1"});

  cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
    std::make_shared<DigitalPID>(step_digital, gains["Kp"], gains["Ki"], gains["Kd"], 0.0));
  parameters.push_back({cntrl, "Digital-2"});


  for(size_t i = 0; i < parameters.size(); ++i) {
    // --- Initialize Data --- //
    pendulum::State state;
    pendulum::LoggingData data;
    data.test_name = parameters[i].second;
    data.data      = std::vector<double>(4);

    ExampleDigital::X_t x{0.0, 0.0}; // Initialize state
    auto plant = ExampleDigital::create(x);

    // ---   Run Test   --- //
    ros::Duration(0.5).sleep();
    ROS_INFO("- Beginning Simulation Loop: %s", parameters[i].second.c_str());
    parameters[i].first->init(x[0]); // Initialize control loop

    double dt = 0.00001; // 0.01 ms
    double t  = 0.0;     // Time since start of test
    double u  = 0.0;     // Input command

    while(ros::ok() && t < kRunTime_s) {
      using namespace std::chrono;

      // Simulation Feedback Loop
      double target = get_target(t);
      parameters[i].first->set_target(target);

      if(parameters[i].second != "Continuous")
        u = parameters[i].first->update(x[0]);

      /// Simulation plant Loop - multiple times per feedback loop
      int count = 0;
      while(count <= 30) {
        if(parameters[i].second == "Continuous")
          u = parameters[i].first->update(x[0]);


        // Update plant & publish
        x = plant->update(u, dt);
        publish(log_pub, data, state, target, u, x, t);

        // Loop counters
        t += dt;
        count++;
        usleep(1000);
      }
    }
  }

  ROS_INFO("--- End of Simulation Loop ---");

  while(!util::check_logging_done(nh)) {
    usleep(1000);
  }
  util::stop_logging(nh, log_table);

  ros::shutdown();
  return 0;
}

void publish(ros::Publisher &log_pub,
  pendulum::LoggingData &data, pendulum::State &state,
  const double target, const double u, const ExampleDigital::X_t &x, const double t)
{
  auto timestamp = ros::Time::now();

  data.header.seq += 1;
  data.header.stamp = timestamp;
  data.data[0] = t;
  data.data[1] = target;
  data.data[2] = u;
  data.data[3] = x[0];
  data.data[4] = x[1];
  log_pub.publish(data);

  ros::spinOnce();
}

double get_target(const double test_time)
{
  double target = 0.0;
  if(test_time < 1.5)
    target = 0.0;
  else
    target = 0.3;

  return 1.0;
}
