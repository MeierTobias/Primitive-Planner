#ifndef _SIMPLE_DRONE_COUNTER_H_
#define _SIMPLE_DRONE_COUNTER_H_

#include <primitive_planner/SimpleCountDrones.h>
#include "drone_counter.h"
#include <std_msgs/Empty.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <memory>

namespace primitive_planner
{

class SimpleDroneCounter : public DroneCounter
{
public:
  void init(ros::NodeHandle &nh, const Eigen::Vector3d &position, unsigned int drones_total, unsigned int drone_id);

  void setReachedGoal() final override;
  void unsetReachedGoal() final override;

  void waitForNDrones(unsigned int n) override;

private:
  enum
  {
    NOT_AT_GOAL,
    ARRIVED_AT_GOAL,
    BROADCASTING,
    NOT_BROADCASTING,
  } state;

  ros::Timer broadcast_timer_;
  ros::Timer wait_for_other_drones_timeout_;

  void countMessageCallback(const primitive_planner::SimpleCountDrones &msg);
  void debugMessageCallback(const std_msgs::Empty &msg);
  void heartbeatCallback(const ros::TimerEvent &heartbeat);
  void waitForOtherDronesTimeoutCallback(const ros::TimerEvent &timeout);

  void sendMessage();

public:
  typedef std::unique_ptr<SimpleDroneCounter> Ptr;
};

} // namespace primitive_planner

#endif
