#ifndef _DRONE_COUNTER_H_
#define _DRONE_COUNTER_H_

#include <std_msgs/Empty.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <Eigen/Core>

namespace primitive_planner
{

class DroneCounter
{
public:
  void init(ros::NodeHandle &nh, const Eigen::Vector3d &position, double drone_com_r, unsigned int drones_total, unsigned int drone_id);

  virtual void setReachedGoal(const Eigen::Vector3d &goal) = 0;
  virtual void unsetReachedGoal() = 0;

  bool allDronesArrived() const
  {
    return drones_at_goal >= drones_total;
  }
  virtual void waitForNDrones(unsigned int n) = 0;
  // Waits until allDronesArrived is true
  void waitForAllDrones()
  {
    waitForNDrones(drones_total);
  }
  unsigned int nbDronesArrived() const
  {
    return drones_at_goal;
  }

protected:
  const Eigen::Vector3d *position = nullptr;
  unsigned int drones_total;
  unsigned int drones_at_goal;
  double drone_com_r;

  ros::Publisher count_pub_;
  ros::Subscriber count_sub_;
  ros::Subscriber debug_sub_;

  void debugMessageCallback(const std_msgs::Empty &msg);

  template <class Message>
  bool comesFromTooFar(const Message &msg)
  {
    Eigen::Vector3d sender_position(msg.position.x, msg.position.y, msg.position.z);
    if ((sender_position - *position).squaredNorm() > drone_com_r * drone_com_r)
      return true;
    if ((sender_position - *position).squaredNorm() < 1e-12) // probably a message from myself
      return true;
    return false;
  }
};

} // namespace primitive_planner

#endif
