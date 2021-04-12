/*
 * File:    RosMotor
 * Author:  pdsherman
 * Date:    April 2021
 *
 * Description: Hardware interface class for full pendulum assembly
 */

#pragma once

#include <external_libs/motor/Motor.hpp>
#include <hardware/encoder/EncoderBoard.hpp>
#include <pendulum/Control.h>

#include <ros/ros.h>

#include <future>


class PendulumHardware
{
public:

// Struct for better readability
// Holds positions of pedulum objects
struct Positions {
  double pendulum_theta; // Angle of pendulum rod
  double position_x;     // Distance of linear base object
};

// Constructor
PendulumHardware(const ros::NodeHandle &nh);

// Destructor
~PendulumHardware(void) = default;

/// Connect to and configure hardware (motor + encoders)
/// @return True if initialization successfull
bool initialize(void);

/// Run cycle of the pendulum hardware
///  - Command motor
///  - Read positions
///  - Publish data
/// @param [in] input_N Desired input force in Newtons
Positions update(const double input_N);

/// Read assembly positions from encoder board
/// @return Positions struct
Positions positions_read(void);

/// Enable drive mode on motor
/// @return True if successful
bool motor_enable(void);

/// Disable the motor drive mode
void motor_disable(void);

/// Check motor status to see if error is present
/// @return True if no errors in status register
bool motor_status_ok(void);

/// Drive the motor at a target current
/// @param [in] current_A Target current in amps
void motor_drive_current(const double current_A);

/// Read the motor current.
/// @return Current (amps) through motor
double motor_read_current(void);

/// Check positive limit switch
/// @return True if switch active
bool motor_lim_pos_check(void);

/// Check negative limit switch
/// @return True if switch active
bool motor_lim_neg_check(void);

private:

/// Converts desired force to required motor current
/// @param [in] force Desired force (Newtons)
/// @return Required motor current (Amps)
double force_to_current(const double force) const;

// ----  ROS member variables  ---- //

/// ROS node handle object
ros::NodeHandle _nh;

/// Publisher for measured positions
ros::Publisher _pub_position;

/// Value to publish
pendulum::Control _msg;

// ----      Concurrency      ---- //

/// Future object for asynchronos read of encoder
std::future<Positions> _pos_fut;

// ----  Physical Hardware    ---- //

/// Interface to physical motor
Motor _mtr;

/// Encoder board driver
EncoderBoard _encoder_board;

/// Hardware is initialized
bool _initialized;

// ----         Constants          ---- //

static const std::string kEncoderBrdPort; ///< Port for i2c device
static constexpr int kEncoderBrdAddr = 0x33; ///< I2C address for encoder board

static constexpr double PI = 3.14159265;          /// This is PI
static constexpr double kPullyRadius_mm = 13.275; /// Radius of pully

};
