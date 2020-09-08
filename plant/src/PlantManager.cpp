
#include <plant/PlantManager.hpp>
#include <plant/Model.hpp>
#include <plant/SimplePendulum.hpp>

PlantManager::PlantManager(
  ros::NodeHandle &nh,
  const std::string &name,
  const State &x0,
  const double u0,
  const double dt,
  const Type plant_type
) :
  _name(name),
  _state(),
  _u(),
  _plant(nullptr),
  _dt(dt),
  _state_pub(nh.advertise<pendulum::State>(name, 500)),
  _control_pub(nh.advertise<pendulum::Control>("control", 500))
{
  switch(plant_type) {
  case Type::Simple:
    _plant = SimplePendulum::create({x0.x, x0.x_dot, x0.theta, x0.theta_dot});
    break;
  case Type::Full:
  default:
    _plant = Model::create({x0.x, x0.x_dot, x0.theta, x0.theta_dot});
    break;
  }

  _u.header.stamp = _test_start_time;
  _u.header.seq   = 0;
  _u.u            = 0.0;

  _state.header.seq   = 0;
  _state.x            = x0.x;
  _state.x_dot        = x0.x_dot;
  _state.theta        = x0.theta;
  _state.theta_dot    = x0.theta_dot;
  _state.test_time_s  = 0.0;
}

void PlantManager::initialize_test(const ros::Time &start_time)
{
  _test_start_time = start_time;

  _u.header.stamp = _test_start_time;
  _control_pub.publish(_u);

  _state.header.stamp = _test_start_time;
  _state_pub.publish(_state);
}

PlantManager::State PlantManager::get_current_state(void) const
{
  return _state;
}

std::shared_ptr<Plant<4>> PlantManager::get_plant(void)
{
  return _plant;
}

void PlantManager::set_control(double u)
{
  _u.u = u;
}

void PlantManager::cycle(const ros::Time &time)
{
  //
  // Update model
  //
  std::array<double, 4> x = _plant->update(_u.u, _dt);

  _state.x = x[0];
  _state.x_dot = x[1];
  _state.theta = x[2];
  _state.theta_dot = x[3];

  //
  // Publish topics
  //
  _state.header.seq += 1;
  _state.header.stamp = time;
  _state.test_time_s = (time - _test_start_time).toSec();
  _state_pub.publish(_state);

  _u.header.seq += 1;
  _u.header.stamp = time;
  _control_pub.publish(_u);
}
