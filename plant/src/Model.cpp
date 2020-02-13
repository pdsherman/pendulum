
#include <plant/Model.hpp>

Model::Model(ros::NodeHandle &nh, const std::string &name, double x0, double theta0) :
  _name(name), _state(), _velocity(), _u(0.0),
  _state_pub(nh.advertise<pendulum::State>(name, 50)),
  _control_sub(nh.subscribe("control", 100, &Model::set_control, this))
{
  _state.header.seq = 0;
  _state.x = x0;
  _state.theta = theta0;
}

Model::State Model::get_current_state(void) const
{
  return _state;
}

void Model::publish(const ros::Time &time)
{
  _state.header.seq += 1;
  _state.header.stamp = time;

  _state_pub.publish(_state);
}

void Model::update(const double h)
{
  std::array<double, 4> kx, kt, fx, ft;

  for(int ii = 0; ii < 4; ++ii) {
    double dx, dt, ddx, ddt;
    switch(ii){
      case 0:
        dx  = 0.0;
        dt  = 0.0;
        ddx = 0.0;
        ddt = 0.0;
      case 1:
        dx  = kx[0]/2.0;
        dt  = kt[0]/2.0;
        ddx = fx[0]/2.0;
        ddt = ft[0]/2.0;
        break;
      case 2:
        dx  = kx[1]/2.0;
        dt  = kt[1]/2.0;
        ddx = fx[1]/2.0;
        ddt = ft[1]/2.0;
        break;
      case 3:
        dx  = kx[2];
        dt  = kt[2];
        ddx = fx[2];
        ddt = ft[2];
        break;
    }

    Acceleration accel = calculate_accel(add_term(_state, dx, dt), add_term(_velocity, ddx, ddt));
    fx[ii] = h*accel.x;
    ft[ii] = h*accel.theta;

    kx[ii] = h*(_velocity.x + ddx);
    kt[ii] = h*(_velocity.theta + ddt);
  }

  // Combine and update states
  double dx     = (kx[0] + 2*kx[1] + 2*kx[2] + kx[3])/6.0;
  double dtheta = (kt[0] + 2*kt[1] + 2*kt[2] + kt[3])/6.0;

  double dx_dot     = (fx[0] + 2*fx[1] + 2*fx[2] + fx[3])/6.0;
  double dtheta_dot = (ft[0] + 2*ft[1] + 2*ft[2] + ft[3])/6.0;

  _state    = add_term(_state, dx, dtheta);
  _velocity = add_term(_velocity, dx_dot, dtheta_dot);
}

Model::Acceleration Model::calculate_accel(const State &state, const Velocity &velocity) const
{
  Acceleration accel;

  // Constant values used in calculations
  static constexpr double m_M  = m + M;
  static constexpr double ll   = l*l;
  static constexpr double ml   = m*l;
  static constexpr double I_ = I + ll;

  // Calculate acceleration of x
  double k1 = -b_x*velocity.x;
  double k2 = l*pow(velocity.x, 2.0)*cos(state.theta);
  double k3 = -m*ll*(b_t*velocity.theta + g*cos(state.theta))*sin(state.theta)/I_ + _u;
  double k4 = m_M - m*ll*pow(sin(state.theta), 2.0)/I_;
  accel.x = (k1 + k2 + k3)/k4;

  // Calculate rotational acceleration of theta
  double k5 = ml/I_;
  double k6 = accel.x*sin(state.theta) - g*cos(state.theta) - b_t*velocity.theta;
  accel.theta = k5*k6;

  return accel;
}

Model::State Model::add_term(State state, const double dx, const double dt) const
{
  state.x += dx;
  state.theta += dt;
  return state;
}

void Model::set_control(const pendulum::Control::ConstPtr &msg)
{
  _u = msg->u;
}
