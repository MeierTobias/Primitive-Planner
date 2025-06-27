#include "plan_manage/bitmask_drone_counter.h"
#include <ros/duration.h>
#include <algorithm>

using namespace primitive_planner;

void BitmaskDroneCounter::init(ros::NodeHandle &nh, const Eigen::Vector3d &position, double drone_com_r, unsigned int drones_total, unsigned int drone_id)
{
  isAtGoal_ = false;
  DroneCounter::init(nh, position, drone_com_r, drones_total, drone_id);

  count_pub_ = nh.advertise<primitive_planner::BitmaskCountDrones>("/distributed_count", 100);
  count_sub_ = nh.subscribe<const primitive_planner::BitmaskCountDrones &>("/distributed_count", 100, &BitmaskDroneCounter::countMessageCallback, this);

  broadcast_timer_ = nh.createTimer(ros::Duration(0.5), &BitmaskDroneCounter::broadcastTimeCallback, this);
  broadcast_timer_.stop();

  bitmask.resize(drones_total, false);
  this->drone_id = drone_id;
}

void BitmaskDroneCounter::sendMessage()
{
  BitmaskCountDrones message;

  message.position.x = (*position)(0);
  message.position.y = (*position)(1);
  message.position.z = (*position)(2);

  message.goal_tag = goal_tag;

  // We're translating from bool to uint8_t (ROS doesn't have bools), so we can't just use operator=
  message.bitmask.resize(bitmask.size());
  std::copy(bitmask.begin(), bitmask.end(), message.bitmask.begin());

  lastBroadcastTime = ros::Time::now();
  count_pub_.publish(message);
}

void BitmaskDroneCounter::countMessageCallback(const primitive_planner::BitmaskCountDrones &msg)
{
  if (comesFromTooFar(msg))
    return;
  if (msg.goal_tag != goal_tag)
    return;

  bool changed = false;
  for (unsigned int i = 0; i < drones_total; i++)
  {
    if (msg.bitmask[i] && !bitmask[i])
    {
      drones_at_goal += 1;
      changed = true;
      bitmask[i] = true;
    }
  }
  ros::Duration timeSinceLastBroadcast = ros::Time::now() - lastBroadcastTime;
  if (changed && timeSinceLastBroadcast < ros::Duration(0.5))
  {
    sendMessage();
  }
}

void BitmaskDroneCounter::setReachedGoal(int goal_tag)
{
  if (!isAtGoal_)
  {
    isAtGoal_ = true;
    this->goal_tag = goal_tag;
    std::fill(bitmask.begin(), bitmask.end(), 0);
    bitmask[drone_id] = true;
    drones_at_goal = 1;
    broadcast_timer_.start();
  }
}

void BitmaskDroneCounter::unsetReachedGoal()
{
  isAtGoal_ = false;
  broadcast_timer_.stop();
}

void BitmaskDroneCounter::broadcastTimeCallback(const ros::TimerEvent &heartbeat)
{
  sendMessage();
}

void BitmaskDroneCounter::waitForNDrones(unsigned int n)
{
  ROS_DEBUG("[COUNT] Waiting for %d drones...", n);
  ros::Rate r(1); // 1hz
  while (drones_at_goal < n)
  {
    ros::spinOnce();
    r.sleep();
  }
}