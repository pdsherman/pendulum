

#include <devices/motor/RosMotor.hpp>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "ros_motor");
  ros::NodeHandle nh;
  RosMotor mtr(nh, "benchtop");

  ROS_INFO("Starting ROS motor node....");
  if(mtr.setup()) {
    ROS_INFO("Motor amp configured.");
    ros::spin();
  } else {
      ROS_WARN("Failed to configure motor amp.");
  }
}
