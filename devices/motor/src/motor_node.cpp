

#include <devices/motor/RosMotor.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_motor");
  ros::NodeHandle nh;
  RosMotor mtr(nh, "benchtop");

  mtr.setup();
  ros::spin();
}
