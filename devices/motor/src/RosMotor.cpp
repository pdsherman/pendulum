/*
 * See header file for description
 */

#include <devices/motor/RosMotor.hpp>

#include <boost/function.hpp>

RosMotor::RosMotor(const ros::NodeHandle &nh, const std::string &name)
 :  _mtr(), _mtr_enabled(false), _nh(nh, name), _mtx_mtr(), _target_current(0.0), _quit(false)
{
  // Startup ROS service server,publisher and subscriber
  _drive_mode_server       = _nh.advertiseService("drive_mode", &RosMotor::set_drive_mode, this);
  _pub_measured_current    = _nh.advertise<pendulum::Current>("current_measured", 50);
  _sub_commanded_current   = _nh.subscribe("current_cmd", 10, &RosMotor::cmd_current_cb, this);
}

RosMotor::~RosMotor(void)
{
  disable_drive_mode();
}

bool RosMotor::setup(void)
{
  std::lock_guard<std::mutex> lock(_mtx_mtr);
  _mtr.configure();
  return true;
}

bool RosMotor::enable_drive_mode(void)
{
  if(_mtr_enabled)
    return false;

  {
    std::lock_guard<std::mutex> lock(_mtx_mtr);
    _mtr.enable();
  }

  _quit = false;
  _mtr_cmd_thread  = std::thread(&RosMotor::motor_writing_thread, this);
  _mtr_read_thread = std::thread(&RosMotor::motor_reading_thread, this);

  _mtr_enabled = true;
  return true;
}

void RosMotor::disable_drive_mode(void)
{
  _quit = true;
  if(_mtr_cmd_thread.joinable())
    _mtr_cmd_thread.join();
  if(_mtr_read_thread.joinable())
    _mtr_read_thread.join();

  {
    std::lock_guard<std::mutex> lock(_mtx_mtr);
    _mtr.disable();
  }

  _mtr_enabled = false;
}

void RosMotor::drive_current(const double current_A)
{
  std::lock_guard<std::mutex> lock(_mtx_mtr);
  _mtr.commanded_current(current_A);
}

double RosMotor::read_current(void)
{
  std::lock_guard<std::mutex> lock(_mtx_mtr);
  return _mtr.read_current();
}

bool RosMotor::motor_has_error(void)
{
  return false;
}

bool RosMotor::set_drive_mode(pendulum::MotorControl::Request &req, pendulum::MotorControl::Response &res)
{
  if(req.active) {
    enable_drive_mode();
    res.success = _mtr_enabled;
  } else {
    disable_drive_mode();
    res.success = !_mtr_enabled;
  }

  return true;
}

void RosMotor::cmd_current_cb(const pendulum::Current::ConstPtr &msg)
{
  _target_current.store(msg->current_A);
}

void RosMotor::motor_writing_thread(void)
{
  static double current_cmd = 0.0;

  while(!_quit) {
    double current_stored = _target_current.load();
    if(current_stored != current_cmd) {
      {
        std::lock_guard<std::mutex> lck(_mtx_mtr);
        _mtr.commanded_current(current_stored);
      }
      current_cmd = current_stored;
    }
  }

  drive_current(0.0);
}

void RosMotor::motor_reading_thread(void)
{
  pendulum::Current msg;

  while(!_quit) {
    {
      std::lock_guard<std::mutex> lck(_mtx_mtr);
      msg.current_A = _mtr.read_current();
    }
    msg.header.stamp = ros::Time::now();
    msg.header.seq  += 1;
    _pub_measured_current.publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
