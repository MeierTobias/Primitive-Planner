#ifndef _DRONE_COUNTER_H_
#define _DRONE_COUNTER_H_

#include <primitive_planner/CountDrones.h>
#include <std_msgs/Empty.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <memory>

namespace primitive_planner
{

class DroneCounter
{
public:
  void init(ros::NodeHandle &nh, const Eigen::Vector3d &position, unsigned int drones_total);

  void setReachedGoal();
  void unsetReachedGoal();

  bool allDronesArrived() const;
  void waitForNDrones(unsigned int n);
  // Waits until allDronesArrived is true
  void waitForAllDrones()
  {
    waitForNDrones(drones_total);
  }
  unsigned int nbDronesArrived() const
  {
    return drones_at_goal;
  }

private:
  const Eigen::Vector3d *position = nullptr;
  unsigned int drones_total;

  unsigned int drones_at_goal;
  enum
  {
    NOT_AT_GOAL,
    ARRIVED_AT_GOAL,
    BROADCASTING,
    NOT_BROADCASTING,
  } state;

  ros::Publisher count_pub_;
  ros::Subscriber count_sub_;
  ros::Subscriber debug_sub_;
  ros::Subscriber debug_sub_2_;
  ros::Timer broadcast_timer_;
  ros::Timer wait_for_other_drones_timeout_;

  void countMessageCallback(const primitive_planner::CountDrones &msg);
  void debugMessageCallback(const std_msgs::Empty &msg);
  void heartbeatCallback(const ros::TimerEvent &heartbeat);
  void waitForOtherDronesTimeoutCallback(const ros::TimerEvent &timeout);

  void sendMessage();

public:
  typedef std::unique_ptr<DroneCounter> Ptr;
};

} // namespace primitive_planner

#endif
