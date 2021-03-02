/*
 * File:    RosMotor
 * Author:  pdsherman
 * Date:    Feb. 2021
 *
 * Description: Wrapper class to add ROS functionality to motor
 */

#pragma once

#include <pendulum/Current.h>
#include <pendulum/MotorControl.h>

#include <external_libs/motor/Motor.hpp>
#include <ros/ros.h>

#include <thread>
#include <mutex>
#include <atomic>

/// Wrapper class for physical motor object. Adds ROS
/// funcionality for motor object such as publishing &
/// subscribing to ROS topics, running ROS servcies
/// and logging.
class RosMotor
{
public:
  /// Constructor
  /// @param [in] nh Parent ROS node handle object
  /// @param [in] name Component name to use for motor
  RosMotor(const ros::NodeHandle &nh, const std::string &name);

  /// Default destructor
  ~RosMotor(void);

  /// Connects to and configures physical motor
  /// @return True if setup successfull
  bool setup(void);

  /// Set motor mode to "drive"
  /// @return True if motor has no errors after switching modes
  bool enable_drive_mode(void);

  /// Set motor mode to "disable"
  /// Results in motor put into a torque free state
  void disable_drive_mode(void);

  /// Check if motor controller is in error state
  /// @return True if status has error
  bool motor_has_error(void);

  /// Set the commanded current for the motor
  /// @param [in] current_A Target current (Amps) to drive motor
  void drive_current(const double current_A);

  /// Read the current (Amps) driving motor
  /// @return Current read by motor controller
  double read_current(void);

private:

  bool set_drive_mode(pendulum::MotorControl::Request &req, pendulum::MotorControl::Response &res);

  void cmd_current_cb(const pendulum::Current::ConstPtr &msg);

  void motor_writing_thread(void);

  void motor_reading_thread(void);

  /// Interface to physical motor
  Motor _mtr;

  /// Is motor already enabled??
  bool _mtr_enabled;

  // --------------------------------- //
  // - ROS specific member variables - //
  // -------------------------------- //

  /// ROS node handle object
  ros::NodeHandle _nh;

  /// Server for motor drive mode service
  ros::ServiceServer _drive_mode_server;

  /// Subscriber for commanded current values
  ros::Subscriber _sub_commanded_current;

  /// Publisher for measured current values
  ros::Publisher _pub_measured_current;

  // ------------------------------------ //
  // - Multi-threading member variables - //
  // ------------------------------------ //

  /// Mutex to synchornize access to motor object
  std::mutex _mtx_mtr;

  /// Thread to ansynchronously write commanded current to motor
  std::thread _mtr_cmd_thread;

  /// Thread to ansynchronously read motor current and publish
  std::thread _mtr_read_thread;

  /// Target current to write to motor
  std::atomic<double> _target_current;

  /// Flag to signal to motor thread to end
  std::atomic<bool> _quit;

}; // class RosMotor
