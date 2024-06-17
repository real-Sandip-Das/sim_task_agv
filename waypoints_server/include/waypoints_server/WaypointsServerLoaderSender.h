/**
 * @file WaypointsServerLoaderSender.h
 * @author Sandip Das (sd13535@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-15
 * 
 * @copyright Copyright (c) 2024 Sandip Das
 * 
 */

#ifndef WAYPOINTS_SERVER_WAYPOINTSSERVERLOADERSENDER_H
#define WAYPOINTS_SERVER_WAYPOINTSSERVERLOADERSENDER_H

#pragma once

// STD
#include <vector>
#include <string>
#include <memory>

// ROS
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

namespace waypoints_server
{

/*!
 * Main class for the node to handle the ROS interfacing.
 */
class WaypointsServerLoaderSender
{
 public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  explicit WaypointsServerLoaderSender(ros::NodeHandle& nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~WaypointsServerLoaderSender();

  /*!
   * Send Waypoints to navigation stack sequentially.
   */
  void sendWaypoints();

 private:
  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */
  bool readParameters();

  /*!
   * Reads waypoints from disk.
   * @return true if successful.
   */
  bool readWaypoints();

  //! Goal callback method for action usage
  void doneCallback(const actionlib::SimpleClientGoalState& state,
                    const boost::shared_ptr<const move_base_msgs::MoveBaseResult> &result);

  //! ROS node handle.
  ros::NodeHandle& nodeHandle_;

  //! Path to file containing waypoints.
  std::string waypointsFilePath_;

  //! Action to send waypoints as goals to.
  std::string moveBaseAction_;

  //! Waypoints read from disk.
  std::vector<move_base_msgs::MoveBaseGoal> waypoints_;

  //! unique_ptr to ROS Action client for sending waypoints as goals to move_base action.
  std::unique_ptr<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>> actionClient_p_;

  //! Serial number of the goal sent.
  size_t goalID_ = 0;
};

} /* namespace waypoints_server */

#endif  // WAYPOINTS_SERVER_WAYPOINTSSERVERLOADERSENDER_H
