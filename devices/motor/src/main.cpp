
#include <pendulum/Current.h>
#include <pendulum/MotorControl.h>

#include <external_libs/motor/Motor.hpp>

#include <ros/ros.h>
#include <boost/function.hpp>

#include <string>
#include <iostream>
//#include <functional>


bool mtr_state(pendulum::MotorControl::Request &req, pendulum::MotorControl::Response &res, std::shared_ptr<Motor> mtr);

void mtr_control(const pendulum::Current::ConstPtr &msg, std::shared_ptr<Motor> mtr);

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "motor");
  ros::NodeHandle nh;

  std::shared_ptr<Motor> mtr = std::make_shared<Motor>();

  boost::function<bool(pendulum::MotorControl::Request &, pendulum::MotorControl::Response &)>
  srv_cb = boost::bind(mtr_state, _1, _2, mtr);
  ros::ServiceServer motor_control_server = nh.advertiseService("control", srv_cb);

  boost::function<void(const pendulum::Current::ConstPtr &msg)> msg_cb = boost::bind(mtr_control, _1, mtr);
  ros::Subscriber subscriber = nh.subscribe("current_cmd", 10, msg_cb);

  ros::Publisher pubber = nh.advertise<pendulum::Current>("current_measured", 100);


  pendulum::Current current;
  ros::Rate rate(1/0.002);
  while(ros::ok()) {
    current.header.stamp = ros::Time::now();
    current.header.seq += 1;
    current.current_A = (current.header.seq % 50)/10.0; //mtr->read_current();
    pubber.publish(current);

    ros::spinOnce();
    rate.sleep();
  }

  return 0;
}

bool mtr_state(pendulum::MotorControl::Request &req, pendulum::MotorControl::Response &res, std::shared_ptr<Motor> mtr)
{
  if(req.active) {
    std::cout << "Enable" << std::endl;
    // mtr->enable();
  } else {
    std::cout << "Disable" << std::endl;
    // mtr->disable();
  }

  res.success = true;

  return true;
}

void mtr_control(const pendulum::Current::ConstPtr &msg, std::shared_ptr<Motor> mtr)
{
  std::cout << "Commanded Current: " << std::to_string(msg->current_A) << std::endl;
  //mtr->commanded_current(msg->current_A);
}
