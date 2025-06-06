#include "plan_manage/drone_counter.h"
#include <std_msgs/Empty.h>
#include <primitive_planner/CountDrones.h>
#include <ros/callback_queue.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/console_backend.h>
#include <ros/init.h>
#include <Eigen/Core>
#include <Eigen/StdVector>

namespace primitive_planner
{

void SimpleDroneCounter::init(ros::NodeHandle &nh, const Eigen::Vector3d &position, unsigned int drones_total)
{
  this->position = &position;
  ROS_DEBUG("[COUNT] init");
  count_pub_ = nh.advertise<primitive_planner::CountDrones>("/distributed_count", 100);
  count_sub_ = nh.subscribe<const primitive_planner::CountDrones &>("/distributed_count", 100, &SimpleDroneCounter::countMessageCallback, this);
  broadcast_timer_ = nh.createTimer(ros::Duration(0.5), &SimpleDroneCounter::heartbeatCallback, this);
  broadcast_timer_.stop();
  wait_for_other_drones_timeout_ = nh.createTimer(ros::Duration(2), &SimpleDroneCounter::waitForOtherDronesTimeoutCallback, this, true);
  wait_for_other_drones_timeout_.stop();

  this->drones_total = drones_total;

  state = NOT_AT_GOAL;
}

void SimpleDroneCounter::setReachedGoal()
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

constexpr double MAX_RECV_RADIUS = 10;

void SimpleDroneCounter::countMessageCallback(const primitive_planner::CountDrones &msg)
{
  Eigen::Vector3d sender_position(msg.position.x, msg.position.y, msg.position.z);

  if ((sender_position - *position).norm() > MAX_RECV_RADIUS)
    return;
  if ((sender_position - *position).norm() < 1e-6) // probably a message from myself
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

bool SimpleDroneCounter::allDronesArrived() const
{
  return drones_at_goal >= drones_total;
}

void SimpleDroneCounter::sendMessage()
{
  primitive_planner::CountDrones message;

  message.position.x = (*position)(0);
  message.position.y = (*position)(1);
  message.position.z = (*position)(2);
  message.drones_at_goal = drones_at_goal;

  count_pub_.publish(message);
}

} // namespace primitive_planner
