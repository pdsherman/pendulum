
#include <pendulum/LoggingStart.h>
#include <pendulum/LoggingStop.h>
#include <pendulum/LoggingDropTable.h>

#include <pendulum/DrawSystem.h>

#include <libs/util/ros_util.hpp>

namespace util {

bool start_logging(ros::NodeHandle &nh,
                  const std::string &table,
                  const std::string &topic,
                  const std::vector<std::string> &headers)
{
  // Request logging node to start logging
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingStart>("/sqlite/start_log");
  if(sql_client.exists()) {
    pendulum::LoggingStart start_log;
    start_log.request.table_name = table;
    start_log.request.topic_name = topic;
    start_log.request.header     = headers;

    if(sql_client.call(start_log) && start_log.response.success) {
      // Experimentally found that some delay is needed after starting
      // logging before you can start publishing data.
      usleep(300000);
      return true;
    }
  } else {
    ROS_WARN("Start logging server unreachable.");
  }

  return false;
}

bool stop_logging(ros::NodeHandle &nh, const std::string &table)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingStop>("/sqlite/stop_log");
  if(sql_client.exists()) {
    pendulum::LoggingStop start_log;
    start_log.request.table_name = table;
    sql_client.call(start_log);
    return true;
  } else {
    ROS_WARN("Stop logging server unreachable.");
  }
  return false;
}

bool drop_logging_table(ros::NodeHandle &nh, const std::string &table)
{
  ros::ServiceClient sql_client = nh.serviceClient<pendulum::LoggingDropTable>("/sqlite/drop_table");
  if(sql_client.exists()) {
    pendulum::LoggingDropTable drop_table;
    drop_table.request.table_name = table;
    sql_client.call(drop_table);
    if(!drop_table.response.success)
      ROS_WARN("Failure to drop table");
    else
      return true;
  } else {
    ROS_WARN("Unable to reach drop table service");
  }
  return false;
}

bool display(ros::NodeHandle &nh, const std::string &topic_name,
  const double x0, const double theta0,
  const int img_type,
  const std::string &base_color, const std::string &pendulum_color)
{
  ros::ServiceClient gui_client = nh.serviceClient<pendulum::DrawSystem>("/gui/draw_system");
  if(gui_client.exists()) {
    pendulum::DrawSystem gui_srv;
    gui_srv.request.name  = topic_name;
    gui_srv.request.x     = x0;
    gui_srv.request.theta = theta0;
    gui_srv.request.img_type = img_type;
    gui_srv.request.base_color = base_color;
    gui_srv.request.pendulum_color = pendulum_color;

    gui_client.call(gui_srv);

    return gui_srv.response.result;
  } else {
    ROS_WARN("Unable to reach DrawSystem service.");
  }

  return false;
}




} // namespace util
