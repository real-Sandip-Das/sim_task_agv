/**
 * @file waypoints_server_saver.cpp
 * @author Sandip Das (sd13535@outlook.com)
 * @brief 
 * @version 0.1
 * @date 2024-06-15
 * 
 * @copyright Copyright (c) 2024 Sandip Das
 * 
 */

#include <ros/ros.h>
#include "waypoints_server/WaypointsServerSaver.h"

int main(int argc, char** argv)
{
  ros::init(argc, argv, "waypoints_server_saver");
  ros::NodeHandle nodeHandle("~");

  waypoints_server::WaypointsServerSaver waypointsServerSaver(nodeHandle);

  ros::spin();
  return 0;
}
