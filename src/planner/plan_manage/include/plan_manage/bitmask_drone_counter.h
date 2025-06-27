#ifndef _BITMASK_DRONE_COUNTER_H_
#define _BITMASK_DRONE_COUNTER_H_

#include <primitive_planner/BitmaskCountDrones.h>
#include "drone_counter.h"
#include <std_msgs/Empty.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <memory>
#include <vector>

namespace primitive_planner
{

class BitmaskDroneCounter : public DroneCounter
{
public:
  void init(ros::NodeHandle &nh, const Eigen::Vector3d &position, double drone_com_r, unsigned int drones_total, unsigned int drone_id);

  void setReachedGoal(int goal_tag) final override;
  void unsetReachedGoal() final override;

  void waitForNDrones(unsigned int n) final override;

  bool isAtGoal() const // for debugging
  {
    return isAtGoal_;
  }

  std::string getBitmask() const;

private:
  unsigned int drone_id;
  std::vector<bool> bitmask;
  int goal_tag;
  ros::Time lastBroadcastTime;

  ros::Timer broadcast_timer_;

  bool isAtGoal_;

  void countMessageCallback(const primitive_planner::BitmaskCountDrones &msg);
  void broadcastTimeCallback(const ros::TimerEvent &heartbeat);
  void sendMessage();

public:
  typedef std::unique_ptr<BitmaskDroneCounter> Ptr;
};

} // namespace primitive_planner

#endif
