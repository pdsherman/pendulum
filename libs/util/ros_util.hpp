/*
 * @file:   util.hpp
 * @author: pdsherman
 * @date:   May 2021
 * @brief:  Assortment of useful funtions for
 * common stuff I was doing with ROS applications
 */

#pragma once

#include <pendulum/DrawSystem.h>

#include <ros/ros.h>
#include <vector>
#include <string>

namespace util {

/// Repeatedly check for if the target service exists.
/// Timeout after after 5 seconds.
/// @return True if service exists before timeout
bool service_exists_timeout(ros::ServiceClient &client);

/// Attempt to call the logging node service to
/// begin subscribing to a topic and log data
/// to SQLite database.
/// @param [in] nh      Reference to ROS node handle object
/// @param [in] table   Name of table to create/use in SQLite database
/// @param [in] topic   Name of topic the logging node should subscribe to
/// @param [in] headers Column names for table
/// @return True if logging node service was reachable and succeeded.
bool start_logging(ros::NodeHandle &nh,
                  const std::string &table,
                  const std::string &topic,
                  const std::vector<std::string> &headers);

/// Call logging node service to stop logging data
/// @param [in] nh    Reference to ROS node handle object.
/// @param [in] table Name of table to stop logging to.
bool stop_logging(ros::NodeHandle &nh, const std::string &table);

/// Call logging node service to delete table from database
/// @param [in] nh    Reference to ROS node handle object.
/// @param [in] table Name of table to stop drop.
/// @return True if service called successfully
bool drop_logging_table(ros::NodeHandle &nh, const std::string &table);

/// Call logging node service to check if logging has completed
/// @param [in] nh    Reference to ROS node handle object.
/// @return True if service called and returned is empty
bool check_logging_done(ros::NodeHandle &nh);

/// Attempt to call the Draw System service to display
/// object on the simulation GUI.
/// @param [in] nh ROS node handle object
/// @param [in] topic_name Name of topic to publish system positions (to update image)
/// @param [in] x0 Initial x position
/// @param [in] theta0 Initial theta position
/// @param [in] img_type Image type [PENDULUM or MASS_ONLY]
/// @param [in] base_color Color of object base
/// @param [in] pendulum_color Color of pendulum
/// @return True if service exists and returns successfully
bool draw_image(ros::NodeHandle &nh, const std::string &topic_name,
  const double x0, const double theta0,
  const int img_type = pendulum::DrawSystemRequest::PENDULUM,
  const std::string &base_color = "blue",
  const std::string &pendulum_color = "green");

/// Remove image of system from system display GUI
/// @param [in] nh ROS node handle object
/// @param [in] topic_name Name of topic that indicates was image to remove
/// @return True if image removal successful
bool remove_image(ros::NodeHandle &nh, const std::string &topic_name);

} // namespace util