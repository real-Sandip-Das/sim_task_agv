/**
 * @file WaypointsServerLoaderSender.cpp
 * @author Sandip Das (sd13535@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-15
 * 
 * @copyright Copyright (c) 2024 Sandip Das
 * 
 */

#include "waypoints_server/WaypointsServerLoaderSender.h"

// STD
#include <fstream>
#include <memory>

// ROS
#include <actionlib/client/simple_action_client.h>

namespace waypoints_server
{

WaypointsServerLoaderSender::WaypointsServerLoaderSender(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  if (!readWaypoints())
  {
    ROS_ERROR_STREAM("Could not read waypoints from disk");
    ros::requestShutdown();
  }
  actionClient_p_ =
    std::make_unique<actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>>(moveBaseAction_, true);
  ROS_INFO("Waiting for Action Server");
  actionClient_p_ -> waitForServer();
  ROS_INFO("Successfully launched node.");
}

WaypointsServerLoaderSender::~WaypointsServerLoaderSender()
{
}

bool WaypointsServerLoaderSender::readParameters()
{
  if (!nodeHandle_.getParam("waypoints_file", waypointsFilePath_)) return false;
  if (!nodeHandle_.getParam("move_base_action", moveBaseAction_)) return false;
  return true;
}

bool WaypointsServerLoaderSender::readWaypoints()
{
  std::ifstream waypointsFile(waypointsFilePath_, std::ios::binary);
  if (!waypointsFile.is_open())
  {
    ROS_ERROR_STREAM("Could not open file: " << waypointsFilePath_);
    return false;  // Failure
  }
  // Finding out the size in bytes
  waypointsFile.seekg(0, std::ios::end);
  uint32_t serial_size = waypointsFile.tellg();
  waypointsFile.seekg(0, std::ios::beg);

  // Reading the object from disk
  boost::shared_array<uint8_t> buffer(new uint8_t[serial_size]);
  ros::serialization::IStream stream(buffer.get(), serial_size);
  if (!waypointsFile.read(reinterpret_cast<char*>(buffer.get()), serial_size))
  {
    ROS_ERROR_STREAM("Error reading file.");
    return false;  // Failure
  }
  stream >> waypoints_;
  return true;  // Success
}

void WaypointsServerLoaderSender::sendWaypoints()
{
  for (auto &waypoint : waypoints_)
  {
    waypoint.target_pose.header.stamp = ros::Time::now();
    actionClient_p_ -> sendGoal(waypoint,
                           boost::bind(&WaypointsServerLoaderSender::doneCallback, this, _1, _2),
                           actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleActiveCallback(),
                           actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction>::SimpleFeedbackCallback());
    actionClient_p_ -> waitForResult();
    goalID_++;
  }
  ros::shutdown();
}

void WaypointsServerLoaderSender::doneCallback(const actionlib::SimpleClientGoalState& state,
                                               const boost::shared_ptr<const move_base_msgs::MoveBaseResult> &result)
{
  ROS_INFO_STREAM("Waypoint reached! Serial Number: " << goalID_);
}

} /* namespace waypoints_server */
