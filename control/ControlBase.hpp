/*
  @file ControlBase.hpp
  @author pdsherman
  @date March. 2020

  @brief Header file for pendulum model for simulation
*/

#pragma once

template<typename X, typename U>
class ControlBase
{
public:

  /// Constructor
  ControlBase(void) = default;

  /// Default
  virtual ~ControlBase(void) = default;

  virtual void init(const X &x0){};

  /// Update method for control loop
  /// @param [in] x Current state of the robot
  /// @return Updated value for control
  virtual U update(const X &x){};

  /// Getter method for control
  /// @return Control value u
  U get_u(void) const {
    return _u;
  }

  /// Setter method for state target
  void set_target(const X &target) {
    _target = target;
  }

  /// Setter method for time between control loop cycles
  /// @param [in] dt Time delta for control loop cycle
  void set_delta_time(const double dt)
  {
    _dt = dt;
  }

protected:

  /// Target state for control loop
  X _target;

  /// Latest calculated control value
  U _u;

  /// Time between loop updates
  double _dt;

};
