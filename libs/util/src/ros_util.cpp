
#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingStop.h>
#include <pendulum/LoggingDropTable.h>
#include <pendulum/LoggingBufferCheck.h>

#include <pendulum/DrawSystem.h>
#include <pendulum/DeleteSystem.h>

#include <libs/util/ros_util.hpp>

namespace util {


bool service_exists_timeout(ros::ServiceClient &client, const double timeout_s)
{
  static constexpr double dt = 0.5; // Check every 0.5 seconds
  const int max_count = static_cast<int>(timeout_s/dt);

  int count = 0;
  while(!client.exists()) {
    if (count++ == max_count) { return false; }
    ros::Duration(dt).sleep();
  }
  return true;
}

bool start_logging(ros::NodeHandle &nh,
                  const std::string &table,
                  const std::string &topic,
                  const std::vector<std::string> &headers)
{
  // Request logging node to start logging
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingStart>("/sqlite/start_log");
  if(!service_exists_timeout(sql_client)) {
    ROS_WARN("Start logging server unreachable.");
    return false;
  }

  pendulum::LoggingStart start_log;
  start_log.request.table_name = table;
  start_log.request.topic_name = topic;
  start_log.request.header     = headers;

  if(sql_client.call(start_log) && start_log.response.success) {
    // Experimentally found that some delay is needed after starting
    // logging before you can start publishing data.
    ros::Duration(0.3).sleep();
    return true;
  }

  return false;
}

bool stop_logging(ros::NodeHandle &nh, const std::string &table)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingStop>("/sqlite/stop_log");
  if(!service_exists_timeout(sql_client)) {
    ROS_WARN("Stop logging server unreachable.");
    return false;
  }

  pendulum::LoggingStop start_log;
  start_log.request.table_name = table;
  sql_client.call(start_log);
  return true;
}

bool drop_logging_table(ros::NodeHandle &nh, const std::string &table)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingDropTable>("/sqlite/drop_table");
  if(!service_exists_timeout(sql_client)) {
    ROS_WARN("Unable to reach drop table service.");
    return false;
  }

  pendulum::LoggingDropTable drop_table;
  drop_table.request.table_name = table;
  sql_client.call(drop_table);

  if(!drop_table.response.success) {
    ROS_WARN("Failure to drop table");
    return false;
  }
  return true;
}

bool check_logging_done(ros::NodeHandle &nh)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingBufferCheck>("/sqlite/log_buffer_check");
  if(!service_exists_timeout(sql_client)) {
    ROS_WARN("Unable to reach drop table service.");
    return false;
  }

  pendulum::LoggingBufferCheck buffer_check;
  sql_client.call(buffer_check);
  return buffer_check.response.is_empty;
}

bool draw_image(ros::NodeHandle &nh,
  const std::string &topic_name,
  const int img_type,
  const std::vector<double> &x0,
  const std::vector<std::string> &colors)
{
  ros::ServiceClient gui_client = nh.serviceClient<pendulum::DrawSystem>("/gui/draw_system");
  if(!service_exists_timeout(gui_client)) {
    ROS_WARN("Unable to reach DrawSystem service.");
    return false;
  }

  pendulum::DrawSystem gui_srv;
  gui_srv.request.name     = topic_name;
  gui_srv.request.img_type = img_type;
  gui_srv.request.x        = x0;
  gui_srv.request.color    = colors;

  gui_client.call(gui_srv);

  return gui_srv.response.result;
}

bool remove_image(ros::NodeHandle &nh, const std::string &topic_name)
{
  ros::ServiceClient gui_client = nh.serviceClient<pendulum::DeleteSystem>("/gui/delete_system");
  if(!service_exists_timeout(gui_client)) {
    ROS_WARN("Unable to reach DeleteSystem service.");
    return false;
  }

  pendulum::DeleteSystem gui_srv;
  gui_srv.request.name = topic_name;
  gui_client.call(gui_srv);
  return gui_srv.response.result;
}

} // namespace util
