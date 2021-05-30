/*
  @file:    control_base.cpp
  @author:  pdsherman
  @date:    April. 2021

  @brief: control base object
*/

#include <pendulum/LoggingData.h>
#include <pendulum/State.h>
#include <pendulum/Control.h>

#include <hardware/pendulum_hardware/PendulumHardware.hpp>
#include <libs/control/Pid.hpp>
#include <libs/control/DigitalPid.hpp>
#include <libs/control/LeadLag.hpp>
#include <libs/util/ros_util.hpp>

#include <ros/ros.h>
#include <iostream>
#include <thread>   // For this_thread::sleep_until
#include <chrono>

static constexpr double kRunTime_s  = 30.0;
static constexpr int kUpdateCycleTime_us  = 5000;
static constexpr double kMaxForce_N = 80.0;
// Time in seconds between cycles
static constexpr double step_s = static_cast<double>(kUpdateCycleTime_us) / (1e6);

static constexpr double Kp = 175.0;
static constexpr double Ki = 550.0;
static constexpr double Kd = 0.0;

static constexpr double Zero = 50.0;
static constexpr double Pole = 5.0;
static constexpr double Gain = 40.5;

/// Publish data from cycle
void publish(ros::Publisher &log_pub, ros::Publisher &state_pub,
  pendulum::LoggingData &data, pendulum::State &state,
  const double target, const double x, const double u, const double t);

/// Initialize logging to table for test
ros::Publisher initialize_logging(ros::NodeHandle &nh, const std::string &log_table_name);

/// Get current target position for system
double get_target(const double test_time);

/// ---------------------------------------- ///
/// ----        MAIN FUNCTION           ---- ///
/// ---------------------------------------- ///
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "base_control");
  ros::NodeHandle nh;

  // ---   Start Logging   --- //
  std::string log_table = "BaseControl";
  ros::Publisher log_pub = initialize_logging(nh, log_table);

  // ---  Display on GUI    --- //
  std::string state_topic = "base_control";
  ros::Publisher state_pub = nh.advertise<pendulum::State>(state_topic, 50);
  util::draw_image(nh, state_topic, 0.0, 0.0, pendulum::DrawSystemRequest::MASS_ONLY, "blue");

  // --- Initialize Variables --- //
  std::shared_ptr<Controller<double, double>> cntrl;
  std::string test_name;
  if(argc > 1) {
    if(std::string(argv[1]) == "pid"){
      cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
        std::make_shared<PID>(step_s, Kp, Ki, Kd, 0.0));
      test_name = "PID";
    } else if(std::string(argv[1]) == "dpid"){
      cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
        std::make_shared<DigitalPID>(step_s, Kp, Ki, Kd, 0.0));
      test_name = "digital-PID";
    } else if(std::string(argv[1]) == "lag") {
      cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
        std::make_shared<LeadLag>(step_s, Zero, Pole, Gain, LeadLag::DigitialTransform::kTustins));
      test_name = "lag";
    } else {
      ROS_WARN("Invalid controller type");
      return 0;
    }
  } else {
    ROS_WARN("Need to pick a controller type (pid|dpid|lag)");
    return 0;
  }
  ROS_INFO("Using %s for control", test_name.c_str());

  PendulumHardware::Positions pos;
  PendulumHardware hw;

  pendulum::State state;
  state.x     = 0.0;
  state.theta = 0.0;

  pendulum::LoggingData data;
  data.test_name = test_name;
  data.data      = std::vector<double>(4);

  double x = 0.0; // Initialize Position
  double u = 0.0; // Input command
  double t = 0.0; // Time since start of test

  // ---   Run Test   --- //
  ros::Duration(1.5).sleep();

  if(!hw.initialize() || !hw.motor_enable()) {
    ROS_ERROR("Failed to initialize hardware.");
    return 0;
  }
  ROS_INFO("Hardware Initialized.");
  ROS_INFO("--- Beginning Test ---");

  std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();
  std::chrono::steady_clock::time_point time_now   = time_start;
  std::chrono::steady_clock::time_point time_loop  = time_start;

  // ***************************************** //
  // ****        MAIN TEST LOOP           **** //
  // ***************************************** //
  while(ros::ok() && t < kRunTime_s) {
    /// Calculate end-time of cycle
    using namespace std::chrono;
    time_loop += microseconds(kUpdateCycleTime_us);

    // Feedback Loop
    double target = get_target(t);
    cntrl->set_target(target);
    u   = cntrl->update(x);

    if(u > kMaxForce_N) {
      ROS_WARN("Command Input MAX Saturation");
      u = kMaxForce_N;
    } else if(u < -kMaxForce_N) {
      ROS_WARN("Command Input MIN Saturation");
      u = -kMaxForce_N;
    }

    pos = hw.update(u);
    x   = pos.position_x;

    // Publish
    int dt   = duration_cast<microseconds>(time_now - time_start).count();
    t = static_cast<double>(dt)/1000000.0;
    publish(log_pub, state_pub, data, state, target, x, u, t);

    //   Wait until end of cycle time
    std::this_thread::sleep_until(time_loop);
    time_now  = steady_clock::now();
  }

  ROS_INFO("--- End of Test Loop ---");
  util::stop_logging(nh, log_table);

  return 0;
}

ros::Publisher initialize_logging(ros::NodeHandle &nh, const std::string &log_table_name)
{
  std::string logging_topic = "base_model";
  ros::Publisher log_pub = nh.advertise<pendulum::LoggingData>("/" + logging_topic, 50);

  std::vector<std::string> header({
    "test", "test_time", "target", "input", "x"
  });

  if(!util::drop_logging_table(nh, log_table_name) ||
     !util::start_logging(nh, log_table_name, logging_topic, header)) {
    ROS_WARN("Failed to start logging.");
  }

  return log_pub;
}

void publish(ros::Publisher &log_pub, ros::Publisher &state_pub,
  pendulum::LoggingData &data, pendulum::State &state,
  const double target, const double x, const double u, const double t)
{
  auto timestamp = ros::Time::now();

  state.header.seq += 1;
  state.header.stamp = timestamp;
  state.x = x;
  state_pub.publish(state);

  data.header.seq += 1;
  data.header.stamp = timestamp;
  data.data[0] = t;
  data.data[1] = target;
  data.data[2] = u;
  data.data[3] = x;
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

  return target;
}
