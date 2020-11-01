/*
 * File:    Model.hpp
 * Author:  pdsherman
 * Date:    Aug. 2020
 *
 * Description: Simulate the full pendulum assembly with linear motion
 */

#pragma once

#include <plant/Plant.hpp>

#include <array>
#include <memory>

class Model : public Plant<4>
{
public:
  using State = Plant<4>::State;

  static std::shared_ptr<Plant<4>> create(const State &x0);

  /// Constructor
  /// @param x0 initial state
  Model(const State &x0);

  /// Destructor
  ~Model(void) = default;

  /// Simulate system one timestep in future
  /// @param u Control during time step
  /// @param dt Time-step to simulate
  State update(const double u, const double dt) override;

private:

  /// Calculate the state-variable derivative
  /// @param [in] x Current value of state variable
  /// @param [in] u Value of control variable
  State calculate_x_dot(const State &x, const double u) const;

  // *********************** //
  //   Simulation Constants  //
  // *********************** //

  static constexpr double l   = 0.5;
  static constexpr double M   = 5.5;
  static constexpr double m   = 2.7;
  static constexpr double b_b = 0.5;
  static constexpr double I   = 0.00474;
  static constexpr double b_p = 0.00061;

  static constexpr double g = 9.80665;  //< Acceleration from gravity (m/s^2)

};
