#include "plan_manage/simple_drone_counter.h"
#include <std_msgs/Empty.h>
#include <primitive_planner/SimpleCountDrones.h>
#include <ros/callback_queue.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/console_backend.h>
#include <ros/init.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace primitive_planner
{

void SimpleDroneCounter::init(ros::NodeHandle &nh, const Eigen::Vector3d &position, double drone_com_r, unsigned int drones_total, unsigned int drone_id)
{
  DroneCounter::init(nh, position, drone_com_r, drones_total, drone_id);

  count_pub_ = nh.advertise<primitive_planner::SimpleCountDrones>("/distributed_count", 100);
  count_sub_ = nh.subscribe<const primitive_planner::SimpleCountDrones &>("/distributed_count", 100, &SimpleDroneCounter::countMessageCallback, this);
  broadcast_timer_ = nh.createTimer(ros::Duration(0.5), &SimpleDroneCounter::heartbeatCallback, this);
  broadcast_timer_.stop();
  wait_for_other_drones_timeout_ = nh.createTimer(ros::Duration(2), &SimpleDroneCounter::waitForOtherDronesTimeoutCallback, this, true);
  wait_for_other_drones_timeout_.stop();

  state = NOT_AT_GOAL;
}

void SimpleDroneCounter::setReachedGoal(int goal_tag)
{
  if (state == SimpleDroneCounter::NOT_AT_GOAL)
  {
    state = SimpleDroneCounter::ARRIVED_AT_GOAL;
    wait_for_other_drones_timeout_.setPeriod(ros::Duration(2)); // This is required (recommended?) after a oneshot timer has been run
    wait_for_other_drones_timeout_.start();
    ROS_DEBUG("Reached goal, starting timeout");
  }
}

void SimpleDroneCounter::unsetReachedGoal()
{
  ROS_ASSERT(state == SimpleDroneCounter::NOT_BROADCASTING || state == SimpleDroneCounter::NOT_AT_GOAL);
  state = SimpleDroneCounter::NOT_AT_GOAL;
}

void SimpleDroneCounter::waitForOtherDronesTimeoutCallback(const ros::TimerEvent &_timeout)
{
  if (state == SimpleDroneCounter::ARRIVED_AT_GOAL)
  {
    drones_at_goal = 1;
    state = SimpleDroneCounter::BROADCASTING;
    sendMessage();
    broadcast_timer_.start();
  }
}

void SimpleDroneCounter::heartbeatCallback(const ros::TimerEvent &heartbeat)
{
  ROS_ASSERT(state == SimpleDroneCounter::BROADCASTING);
  sendMessage();
}

void SimpleDroneCounter::countMessageCallback(const primitive_planner::SimpleCountDrones &msg)
{
  if (comesFromTooFar(msg))
    return;

  if (state == SimpleDroneCounter::ARRIVED_AT_GOAL)
  {
    drones_at_goal = msg.drones_at_goal + 1;
    state = SimpleDroneCounter::BROADCASTING;
    sendMessage();
    broadcast_timer_.start();
  }
  else if (state == SimpleDroneCounter::BROADCASTING)
  {
    // In case some other drones have started a count, merge the counts together
    if (msg.drones_at_goal < static_cast<int>(drones_at_goal))
    {
      drones_at_goal += msg.drones_at_goal;
    }
    else if (msg.drones_at_goal > static_cast<int>(drones_at_goal))
    {
      drones_at_goal = msg.drones_at_goal;
      state = SimpleDroneCounter::NOT_BROADCASTING;
      broadcast_timer_.stop();
    }
    else
    {
      // We don't know how to handle this
      // we could merge them together, but then both would continue broadcasting and next time they'll both hear each other again and double their estimate
      ROS_ERROR("[COUNT] Two different counts with the same number!");
    }
  }
  else if (state == SimpleDroneCounter::NOT_BROADCASTING)
  {
    if (msg.drones_at_goal > static_cast<int>(drones_at_goal))
      drones_at_goal = msg.drones_at_goal;
  }
}

void SimpleDroneCounter::waitForNDrones(unsigned int n)
{
  ROS_ASSERT(state != SimpleDroneCounter::NOT_AT_GOAL);
  ROS_DEBUG("[COUNT] Waiting for %d drones...", n);
  ros::Rate r(1); // 1hz
  while (drones_at_goal < n)
  {
    ros::spinOnce();
    r.sleep();
  }
}

void SimpleDroneCounter::sendMessage()
{
  primitive_planner::SimpleCountDrones message;

  message.position.x = (*position)(0);
  message.position.y = (*position)(1);
  message.position.z = (*position)(2);
  message.drones_at_goal = drones_at_goal;

  count_pub_.publish(message);
}

} // namespace primitive_planner
