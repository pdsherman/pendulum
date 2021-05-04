/*
 * See header file for description
 */

#include <hardware/pendulum_hardware/PendulumHardware.hpp>

#include <external_libs/motor/MotorException.hpp>

const std::string PendulumHardware::kEncoderBrdPort = "/dev/i2c-1";

PendulumHardware::PendulumHardware(const ros::NodeHandle &nh) :
  _nh(nh),
  _pub_position(),
  _mtr(),
  _encoder_board(kEncoderBrdPort, kEncoderBrdAddr),
  _initialized(false)
{
  _pub_position = _nh.advertise<pendulum::Control>("control", 100);
}

bool PendulumHardware::initialize(void)
{
  bool motor_ok   = false;
  bool encoder_ok = false;

  try {
    _mtr.configure();
    motor_ok = _mtr.status_ok();
  } catch(const MotorException& e) {
    ROS_WARN("Failed to initialize motor. Error: %s", e.what());
  }

  if(_encoder_board.connect()) {
    _encoder_board.set_mode(EncoderBoard::Mode::Both);
    _encoder_board.zero_position();
    encoder_ok = true;
  } else {
      ROS_WARN("Failed to connect to encoder board");
  }

  return motor_ok && encoder_ok;
}

PendulumHardware::Positions PendulumHardware::update(const double input_N)
{
  double current = force_to_current(input_N);

  // Read encoder board at the same time as writing to motor controller
  _pos_fut = std::async(std::launch::async, &PendulumHardware::positions_read, this);
  motor_drive_current(current);
  Positions pos = _pos_fut.get();

  _msg.theta = pos.pendulum_theta;
  _msg.x     = pos.position_x;
  _msg.u     = input_N;

  _msg.header.stamp = ros::Time::now();
  _msg.header.seq  += 1;
  _pub_position.publish(_msg);

  return pos;
}

PendulumHardware::Positions PendulumHardware::positions_read(void)
{
  std::array<double, 2> readings = _encoder_board.position();

  Positions pos({
    .pendulum_theta = readings[1],
    .position_x     = (readings[0] * kPullyRadius_mm)/1000.0
  });
  return pos;
}

bool PendulumHardware::motor_enable(void)
{
  bool ok = false;

  try {
    _mtr.enable();
    ok = _mtr.status_ok();
  } catch(const MotorException& e) {
    ROS_WARN("Unable to enable motor. Error: %s", e.what());
  }
  return ok;
}

void PendulumHardware::motor_disable(void)
{
  try {
    _mtr.disable();
  } catch(const MotorException& e) {
    ROS_WARN("Unable to disable motor. Error: %s", e.what());
  }
}

bool PendulumHardware::motor_status_ok(void)
{
  return true;
}

void PendulumHardware::motor_drive_current(const double current_A)
{
  try {
    _mtr.commanded_current(current_A);
  } catch(const MotorException& e) {
    ROS_WARN("Error during motor drive command. Error: %s", e.what());
  }
}

double PendulumHardware::motor_read_current(void)
{
  double current = 0.0;
  try {
    current = _mtr.read_current();
  } catch(const MotorException& e) {
    ROS_WARN("Error during motor read current. Error: %s", e.what());
  }
  return current;
}

bool PendulumHardware::motor_lim_pos_check(void)
{
  bool active = true;
  try {
    active =  _mtr.lim_pos_active();
  } catch(const MotorException& e) {
    ROS_WARN("Error during pos lim check. Error: %s", e.what());
  }
  return active;
}

bool PendulumHardware::motor_lim_neg_check(void)
{
  bool active = true;
  try {
    active =  _mtr.lim_neg_active();
  } catch(const MotorException& e) {
    ROS_WARN("Error during neg lim check. Error: %s", e.what());
  }
  return active;
}

double PendulumHardware::force_to_current(const double force) const
{
  static constexpr double slope  = 0.0156;
  static constexpr double offset = 0.1050;

  if(force > 0.0)
    return slope * force + offset;
  else if (force < 0.0)
    return slope * force - offset;

  return 0.0;
}
