/**
 * @file WaypointsServerSaver.cpp
 * @author Sandip Das (sd13535@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-15
 * 
 * @copyright Copyright (c) 2024 Sandip Das
 * 
 */

#include "waypoints_server/WaypointsServerSaver.h"

// STD
#include <vector>
#include <limits>
#include <fstream>
#include <sstream>
#include <algorithm>

// ROS
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <cartographer_ros_msgs/TrajectoryQuery.h>
#include <cartographer_ros_msgs/GetTrajectoryStates.h>
#include <move_base_msgs/MoveBaseGoal.h>

// yaml-cpp
#include <yaml-cpp/yaml.h>

template<>
inline geometry_msgs::PoseStamped tf2::toMsg(const visualization_msgs::Marker& a)
{
  geometry_msgs::PoseStamped out;
  out.header = a.header;
  out.pose = a.pose;
  return out;
}

namespace waypoints_server
{

WaypointsServerSaver::WaypointsServerSaver(ros::NodeHandle& nodeHandle)
    : nodeHandle_(nodeHandle), tfListener_(tfBuffer_)
{
  if (!readParameters())
  {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }
  landmarkSubscriber_ = nodeHandle_.subscribe(landmarkTopic_, 1,
                                      &WaypointsServerSaver::landmarkTopicCallback, this);
  ROS_INFO("Successfully launched node.");
}

WaypointsServerSaver::~WaypointsServerSaver()
{
}

bool WaypointsServerSaver::readParameters()
{
  if (!nodeHandle_.getParam("waypoints_file", waypointsFilePath_)) return false;
  if (!nodeHandle_.getParam("landmark_topic", landmarkTopic_)) return false;
  if (!nodeHandle_.getParam("trajectory_query_service", trajectoryQueryService_)) return false;
  if (!nodeHandle_.getParam("get_trajectory_states", getTrajectoryStatesService_)) return false;
  if (!nodeHandle_.getParam("map_frame", mapFrame)) return false;
  return true;
}

void WaypointsServerSaver::landmarkTopicCallback(const visualization_msgs::MarkerArray& landmarkArray)
{
  // Retrieving trajectory from cartographer_ros's services
  ros::ServiceClient clientTrajectoryQuery =
    nodeHandle_.serviceClient<cartographer_ros_msgs::TrajectoryQuery>(trajectoryQueryService_);
  ros::ServiceClient clientGetTrajectoryStates =
    nodeHandle_.serviceClient<cartographer_ros_msgs::GetTrajectoryStates>(getTrajectoryStatesService_);
  cartographer_ros_msgs::GetTrajectoryStates getTrajectoryStatesCall;
  if (!clientGetTrajectoryStates.call(getTrajectoryStatesCall))
  {
    ROS_ERROR_STREAM("Failed to call service: " << getTrajectoryStatesService_);
    return;  // Failure
  }
  std::vector<std::vector<geometry_msgs::PoseStamped>> trajectories;
  for (size_t i = 0; i < getTrajectoryStatesCall.response.trajectory_states.trajectory_id.size(); ++i)
  {
    cartographer_ros_msgs::TrajectoryQuery trajectoryQueryCall;
    if (getTrajectoryStatesCall.response.trajectory_states.trajectory_state[i] ==
        cartographer_ros_msgs::TrajectoryStates::DELETED)
      continue;
    trajectoryQueryCall.request.trajectory_id = getTrajectoryStatesCall.response.trajectory_states.trajectory_id[i];
    if (!clientTrajectoryQuery.call(trajectoryQueryCall))
    {
      ROS_ERROR_STREAM("Failed to call service: " << trajectoryQueryService_);
      return;  // Failure
    }
    trajectories.push_back(trajectoryQueryCall.response.trajectory);
  }

  // Storing Landmark positions in a vector
  std::vector<geometry_msgs::PoseStamped> landmarksPoseStampedTransformed;
  for (size_t i = 0; i < landmarkArray.markers.size(); i++)
  {
    geometry_msgs::PoseStamped landmarkPoseStampedTransformed, landmarkPoseStamped =
      tf2::toMsg<visualization_msgs::Marker, geometry_msgs::PoseStamped>(landmarkArray.markers[i]);
    try
    {
      landmarkPoseStampedTransformed = tfBuffer_.transform(landmarkPoseStamped, mapFrame);
    }
    catch (tf2::TransformException &ex)
    {
      ROS_WARN("%s", ex.what());
      return;  // Failure
    }
    landmarksPoseStampedTransformed.push_back(landmarkPoseStampedTransformed);
  }

  // Declaring dynamic arrays for storing closest points to each landmark in the trajectory
  std::vector<geometry_msgs::PoseStamped> trajectoryPointClosest(landmarksPoseStampedTransformed.size());
  std::vector<double> minDist2(landmarksPoseStampedTransformed.size(), std::numeric_limits<double>::max());

  for (auto &trajectory : trajectories)
  {
    for (auto &trajectoryPoseStamped : trajectory)
    {
      for (size_t i = 0; i < landmarksPoseStampedTransformed.size(); i++)
      {
        geometry_msgs::PoseStamped trajectoryPoseStampedTransformed;
        try
        {
          trajectoryPoseStampedTransformed = tfBuffer_.transform(trajectoryPoseStamped, mapFrame);
        }
        catch (tf2::TransformException &ex)
        {
          ROS_WARN("%s", ex.what());
          return;  // Failure
        }
        tf2::Vector3 trajectoryPoint, landmarkPoint;
        tf2::convert(trajectoryPoseStampedTransformed.pose.position, trajectoryPoint);
        tf2::convert(landmarksPoseStampedTransformed[i].pose.position, landmarkPoint);
        double dist2 = tf2::tf2Distance2(trajectoryPoint, landmarkPoint);
        if (minDist2[i] > dist2)
        {
          trajectoryPointClosest[i] = trajectoryPoseStampedTransformed;
          minDist2[i] = dist2;
        }
      }
    }
  }

  // Saving to disk
  std::vector<move_base_msgs::MoveBaseGoal> goals;
  for (auto goalPoseStamped : trajectoryPointClosest)
  {
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose = goalPoseStamped;
    goals.push_back(goal);
  }
  auto compSort = [](const move_base_msgs::MoveBaseGoal& lhs, const move_base_msgs::MoveBaseGoal& rhs)
  {
    return lhs.target_pose.header.stamp < rhs.target_pose.header.stamp;
  };
  std::sort(goals.begin(), goals.end(), compSort);
  std::ofstream waypointsFile(waypointsFilePath_, std::ios::out | std::ios::trunc);
  if (waypointsFile.is_open())
  {
    YAML::Emitter out;
    out << YAML::BeginSeq;
    for (auto goal : goals)
    {
      std::stringstream ss;
      ss << goal;
      YAML::Node node = YAML::Load(ss.str());
      out << node;
    }
    out << YAML::EndSeq;
    waypointsFile << out.c_str();
  }
  else
    ROS_ERROR_STREAM("Error opening file: " << waypointsFilePath_);
  waypointsFile.close();

  ros::shutdown();
}

} /* namespace waypoints_server */
