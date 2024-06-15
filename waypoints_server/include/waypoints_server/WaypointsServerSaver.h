/**
 * @file WaypointsServerSaver.h
 * @author Sandip Das (sd13535@outlook.com)
 * @brief
 * @version 0.1
 * @date 2024-06-15
 *
 * @copyright Copyright (c) 2024 Sandip Das
 *
 */

#ifndef WAYPOINTS_SERVER_WAYPOINTSSERVERSAVER_H
#define WAYPOINTS_SERVER_WAYPOINTSSERVERSAVER_H

#pragma once

// STD
#include <string>

// ROS
#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>

template <>
inline geometry_msgs::PoseStamped tf2::toMsg(const visualization_msgs::Marker& a);

namespace waypoints_server
{
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class WaypointsServerSaver
{
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit WaypointsServerSaver(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~WaypointsServerSaver();

private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * ROS topic callback method for receiving landmark positions.
   * @param message the received message.
   */
  void landmarkTopicCallback(const visualization_msgs::MarkerArray& message);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  tf2_ros::Buffer tfBuffer_;
  tf2_ros::TransformListener tfListener_;

  ros::Subscriber landmarkSubscriber_;
  std::string landmarkTopic_;

  std::string trajectoryQueryService_;
  std::string getTrajectoryStatesService_;

  //! Path to the YAML file where waypoints need to be saved
  std::string waypointsFilePath_;

  //! Frame with respect to which landmarks will be saved
  std::string mapFrame;
};
} /* namespace waypoints_server */

#endif  // WAYPOINTS_SERVER_WAYPOINTSSERVERSAVER_H
