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
#include <pendulum/LoggingData.h>
#include <pendulum/EncoderTest.h>

#include <hardware/encoder/EncoderBoard.hpp>
#include <libs/util/ros_util.hpp>

#include <ros/ros.h>
#include <boost/function.hpp>

#include <chrono>
#include <memory>

bool encoder_service(
  pendulum::EncoderTest::Request &req,
  pendulum::EncoderTest::Response &res,
  ros::NodeHandle &nh,
  std::shared_ptr<EncoderBoard> encoder);

bool node_setup(std::shared_ptr<EncoderBoard> encoder, ros::NodeHandle &nh, const std::string &topic_name);

static std::string log_test_name  = "";
static std::string log_table_name = "";
static const std::string log_topic_name   = "logged_encoder_data";
static const std::string state_topic_name = "encoder_one";
static std::chrono::steady_clock::time_point time_start = std::chrono::steady_clock::now();

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "single_encoder");
  ros::NodeHandle nh;

  std::shared_ptr<EncoderBoard> encdr = std::make_shared<EncoderBoard>("/dev/i2c-1", 0x33);
  if(!node_setup(encdr, nh, state_topic_name)) {
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
  ros::Publisher pub_state = nh.advertise<pendulum::State>(state_topic_name, 100);
  pendulum::State state;
  state.x.resize(4, 0.0);
  state.x[0] = 0.4;

  // Publishing object for logger node
  ros::Publisher pub_log = nh.advertise<pendulum::LoggingData>(log_topic_name, 100);
  pendulum::LoggingData log_data;
  log_data.data = std::vector<double>(2);


  ROS_INFO("Starting Loop");
  const double dt = 0.02; // Time step
  ros::Rate rate(1/dt);

  usleep(500000);

  while(ros::ok()) {
    // update state msg
    double pos = encdr->position_single();
    state.x[2] = pos;
	  state.header.seq += 1;
    state.header.stamp = ros::Time::now();

    // update log msg
    using namespace std::chrono;
    steady_clock::time_point time_now = steady_clock::now();
    int dt = duration_cast<microseconds>(time_now - time_start).count();

    log_data.header.seq += 1;
    log_data.header.stamp = ros::Time::now();
    log_data.test_name = log_test_name;
    log_data.data[0] = static_cast<double>(dt)/1000000.0;
    log_data.data[1] = pos;

    // publish and delay
    pub_state.publish(state);
    pub_log.publish(log_data);
    ros::spinOnce();
    rate.sleep();

    // show on console for info sake
    if(state.header.seq % 200 == 0)
      ROS_INFO("Position: %f", pos);
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
  if(!util::draw_image(nh, topic_name, pendulum::DrawSystemRequest::PENDULUM, {0.3, 0.0}, {"red", "green"})) {
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
      {
        log_test_name  = req.test_name;
        log_table_name = req.table_name;
        std::vector<std::string> header({"test_name", "test_time", "theta"});
        if(util::start_logging(nh, log_table_name, log_topic_name, header)) {
          time_start = std::chrono::steady_clock::now();
          ROS_INFO("Logging started to table %s", req.table_name.c_str());
        } else {
          ROS_WARN("Failed to start logging");
        }
      }
      break;
    case pendulum::EncoderTestRequest::STOP_LOGGING:
      if(!log_table_name.empty() && util::stop_logging(nh, log_table_name)) {
        ROS_INFO("Logging stopped for table %s", log_table_name.c_str());
      } else {
        ROS_WARN("Failed to stop logging");
      }
      break;
    case pendulum::EncoderTestRequest::SET_OFFSET:
      ROS_INFO("Setting Offset: %f", req.offset);
      encoder->set_offset(EncoderBoard::Encoder::One, req.offset);
      break;
  }

  return true;
}
