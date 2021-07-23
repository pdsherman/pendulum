/*
  @file:    control_base.cpp
  @author:  pdsherman
  @date:    April. 2021

  @brief: control base object
*/

#include <pendulum/LoggingData.h>
#include <pendulum/State.h>
#include <pendulum/Control.h>

#include <plant/BaseObject.hpp>
#include <libs/control/Pid.hpp>
#include <libs/control/DigitalPid.hpp>
#include <libs/control/LeadLag.hpp>
#include <libs/util/ros_util.hpp>

#include <ros/ros.h>
#include <iostream>
#include <thread>   // For this_thread::sleep_until
#include <chrono>

static constexpr double kRunTime_s  = 10.0;
static constexpr int kUpdateCycleTime_us  = 7500;
static constexpr int kSolverCycleTime_us  = 250;
static constexpr double step = static_cast<double>(kUpdateCycleTime_us) / (1e6);  // Discrete time step between cycles

static constexpr double Kp = 200.0;
static constexpr double Ki = 150.0;
static constexpr double Kd = 5.5;

static constexpr double Zero = 50.0;
static constexpr double Pole = 5.0;
static constexpr double Gain = 40.5;

void publish(ros::Publisher &log_pub, ros::Publisher &state_pub,
  pendulum::LoggingData &data, pendulum::State &state,
  const double target, const double u, const BaseObject::X_t &x, const double t);

double get_target(const double test_time);

/// ---------------------------------------- ///
/// ----        MAIN FUNCTION           ---- ///
/// ---------------------------------------- ///
int main(int argc, char *argv[])
{
  ros::init(argc, argv, "base_control");
  ros::NodeHandle nh;

  // ---  Display on GUI    --- //
  std::string state_topic = "base_simulated";
  util::draw_image(nh, state_topic, 0.0, 0.0, pendulum::DrawSystemRequest::MASS_ONLY, "blue");
  ros::Publisher state_pub = nh.advertise<pendulum::State>(state_topic, 50);

  // ---   Start Logging   --- //
  std::string logging_topic = "base_model";
  ros::Publisher log_pub = nh.advertise<pendulum::LoggingData>("/" + logging_topic, 50);
  std::string log_table = "BaseModelTest";
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

  cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
    std::make_shared<PID>(step, Kp, Ki, 0.0, 0.0));
  parameters.push_back({cntrl, "PID"});

  cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
    std::make_shared<DigitalPID>(step, Kp, Ki, 0.0, 0.0));
  parameters.push_back({cntrl, "digital-PID"});

  // cntrl = std::dynamic_pointer_cast<Controller<double, double>>(
  //   std::make_shared<LeadLag>(step, Zero, Pole, Gain, LeadLag::DigitialTransform::kTustins));
  // parameters.push_back({cntrl, "lag"});


  for(size_t i = 0; i < parameters.size(); ++i) {
    // --- Initialize Data --- //
    pendulum::State state;
    pendulum::LoggingData data;
    data.test_name = parameters[i].second;
    data.data      = std::vector<double>(4);

    BaseObject::X_t x{0.0, 0.0}; // Initialize state
    std::shared_ptr<Plant<2>> plant = BaseObject::create(x);

    // ---   Run Test   --- //
    ros::Duration(0.5).sleep();
    ROS_INFO("- Beginning Simulation Loop: %s", parameters[i].second.c_str());
    parameters[i].first->init(x[0]); // Initialize control loop

    double t = 0.0;   // Time since start of test
    double u = 0.0;   // Input command

    std::chrono::steady_clock::time_point start_time = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point now_time        = start_time;
    std::chrono::steady_clock::time_point outer_stop_time = start_time;
    std::chrono::steady_clock::time_point inner_stop_time = start_time;



    while(ros::ok() && t < kRunTime_s) {
      using namespace std::chrono;
      outer_stop_time += microseconds(kUpdateCycleTime_us); // Control Loop Update Time

      // Simulation Feedback Loop
      double target = get_target(t);
      parameters[i].first->set_target(target);
      u = parameters[i].first->update(x[0]);

      while(now_time <= outer_stop_time) {
        inner_stop_time += microseconds(kSolverCycleTime_us);

        // Update Plant
        int dt = duration_cast<microseconds>(now_time - start_time).count();
        t      = static_cast<double>(dt)/1000000.0;
        x      = plant->update(u, step);

        // Publish;
        publish(log_pub, state_pub, data, state, target, u, x, t);

        //   Wait until end of cycle time
        std::this_thread::sleep_until(inner_stop_time);
        now_time  = steady_clock::now();
      }
    }
  }

  ROS_INFO("--- End of Simulation Loop ---");
  util::stop_logging(nh, log_table);
  util::remove_image(nh, state_topic);

  return 0;
}

void publish(ros::Publisher &log_pub, ros::Publisher &state_pub,
  pendulum::LoggingData &data, pendulum::State &state,
  const double target, const double u, const BaseObject::X_t &x, const double t)
{
  auto timestamp = ros::Time::now();

  state.header.seq += 1;
  state.header.stamp = timestamp;
  state.x = x[0];
  state.theta = 0.0; // No Theta for this one.
  state_pub.publish(state);

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

  return target;
}
