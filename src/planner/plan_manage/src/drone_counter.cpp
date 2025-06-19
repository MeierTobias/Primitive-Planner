#include "plan_manage/drone_counter.h"
#include <ros/node_handle.h>
#include <ros/console.h>

using namespace primitive_planner;

void DroneCounter::init(ros::NodeHandle &nh, const Eigen::Vector3d &position, double drone_com_r, unsigned int total_drones, unsigned int drone_id)
{
  this->position = &position;
  this->drones_total = total_drones;
  this->drone_com_r = drone_com_r;
  ROS_INFO("[COUNT] init, position is %f, %f, %f", position[0], position[1], position[2]);
  debug_sub_ = nh.subscribe<const std_msgs::Empty &>("/debug_count", 100, &DroneCounter::debugMessageCallback, this);
}

void DroneCounter::debugMessageCallback(const std_msgs::Empty &msg)
{
  ROS_INFO("[COUNT]: total %d/%d", drones_at_goal, drones_total);
}

/*
void DroneCounter::waitForNDrones(unsigned int n)
{
  ROS_DEBUG("[COUNT] Waiting for %d drones...", n);
  ros::Rate r(1); // 1hz
  while (drones_at_goal < n)
  {
    ros::spinOnce();
    r.sleep();
  }
}
*/
