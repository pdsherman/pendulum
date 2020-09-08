/*
  File:   PlantManager.hpp
  Author: pdsherman
  Date:   Feb. 2020

  Description: Class responsible for managing the system model and
  publishing/subscribing to ROS topics
*/

#pragma once

#include <pendulum/State.h>
#include <pendulum/Control.h>

#include <plant/Plant.hpp>

#include <ros/ros.h>

#include<memory>

class PlantManager
{
public:
  using State = pendulum::State;

  enum class Type {
    Full,
    Simple
  };

  /// Constructor
  /// @param [in] nh ROS node handle object
  /// @param [in] name Identifying name for pendulum object
  /// @param [in] x0 Initial value of state
  /// @param [in] dt Time-step to use for simulation cycles
  PlantManager(
    ros::NodeHandle &nh,
    const std::string &name,
    const State &x0,
    const double u0,
    const double dt,
    const Type plant_type = Type::Full);

  /// Default destructor
  ~PlantManager(void) = default;

  void initialize_test(const ros::Time &start_time);

  void cycle(const ros::Time &time);

  /// Get the current values for the state position values
  /// @return State [x, theta]
  State get_current_state(void) const;

  /// Method to set control directly.
  /// Don't use if
  void set_control(double u);

  std::shared_ptr<Plant<4>> get_plant(void);

private:

  /// Identifying name for object
  std::string _name;

  /// Current position value for pendulum state.
  /// x: Position of movable base along x-axis (meters)
  /// theta: rotational positoin of pendulum (radians)
  State _state;

  /// Control input value. Force (Newtons) exerted on movable base.
  pendulum::Control _u;

  /// Object to simulate the system
  std::shared_ptr<Plant<4>> _plant;

  /// Time step to use for model simulation
  double _dt;

  /// Simulation test start time
  ros::Time _test_start_time;

  /// Publisher for state value
  ros::Publisher _state_pub;

  /// Publisher for control value
  ros::Publisher _control_pub;

};
