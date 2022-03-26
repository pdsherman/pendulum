/*
 * @file    single_encoder.cpp
 * @author  pdsherman
 * @date    March 2022
 * @brief   Node for reading and publishing encoder position values
 * @note    This node is intended to run on a separate device than the ROS master and
 *          and other nodes. This runs on RaspberryPi connected to encoder reading board.
*/

#include <pendulum/State.h>
#include <pendulum/DrawSystem.h>
#include <pendulum/LoggingStart.h>
#include <pendulum/EncoderTest.h>

#include <hardware/encoder/EncoderBoard.hpp>
#include <libs/util/ros_util.hpp>

#include <ros/ros.h>
#include <boost/function.hpp>

#include <memory>

bool encoder_service(
  pendulum::EncoderTest::Request &req,
  pendulum::EncoderTest::Response &res,
  ros::NodeHandle &nh,
  std::shared_ptr<EncoderBoard> encoder);

bool node_setup(std::shared_ptr<EncoderBoard> encoder, ros::NodeHandle &nh, const std::string &topic_name);

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "single_encoder");
  ros::NodeHandle nh;


  std::shared_ptr<EncoderBoard> encdr = std::make_shared<EncoderBoard>("/dev/i2c-1", 0x33);
  const std::string topic_name = "encoder_one";


  if(!node_setup(encdr, nh, topic_name)) {
    ROS_WARN("Unable to start encoder node.");
    return 1;
  }

  static constexpr double pi  = 3.14159;
  encdr->set_offset(EncoderBoard::Encoder::One, -pi/2);


  // Server to handle encoder test requests
  boost::function<bool(pendulum::EncoderTest::Request &, pendulum::EncoderTest::Response &)>
    service_cb = boost::bind(encoder_service, _1, _2, nh, encdr);
  ros::ServiceServer encoder_server = nh.advertiseService("encoder_test", service_cb);

  // Publishing object for encoder data
  ros::Publisher pub = nh.advertise<pendulum::State>(topic_name, 100);
  pendulum::State state;
  state.x = 0.3; // Not used in this application

  ROS_INFO("Starting Loop");
  const double dt = 0.02; // Time step
  ros::Rate rate(1/dt);

  usleep(500000);

  while(ros::ok()) {
    // update state variable
    double pos = encdr->position_single();
    state.theta = pos;
	  state.header.seq += 1;
    state.header.stamp = ros::Time::now();

    if(state.header.seq % 200 == 0)
      ROS_INFO("Position: %f", pos);

    // publish and delay
    pub.publish(state);
    ros::spinOnce();
    rate.sleep();
  }


  return 0;
}

bool node_setup(std::shared_ptr<EncoderBoard> encoder, ros::NodeHandle &nh, const std::string &topic_name)
{
  // Setup encoder hardware object
  if(!encoder->connect()) {
    ROS_WARN("Couldn't connect to encoder.");
    return false;
  }
  encoder->set_mode(EncoderBoard::Mode::Both);
  ROS_INFO("Encoder setup sucessful");

  // Display object onto GUI
  if(!util::draw_image(nh, topic_name, 0.3, 0.0)) {
      ROS_WARN("Timeout waiting for DrawSystem service to exist.");
  }

  return true;
}

bool encoder_service(
  pendulum::EncoderTest::Request &req,
  pendulum::EncoderTest::Response &res,
  ros::NodeHandle &nh,
  std::shared_ptr<EncoderBoard> encoder)
{
  switch(req.command) {
    case pendulum::EncoderTestRequest::ZERO:
      ROS_INFO("Zero Encoder");
      encoder->zero_position();
      break;
    case pendulum::EncoderTestRequest::START_LOGGING:
      ROS_INFO("Start Logging");
      break;
    case pendulum::EncoderTestRequest::STOP_LOGGING:
      ROS_INFO("Stop Logging");
      break;
    case pendulum::EncoderTestRequest::SET_OFFSET:
      ROS_INFO("Setting Offset");
      encoder->set_offset(EncoderBoard::Encoder::One, req.offset);
      break;
  }

  return true;
}
