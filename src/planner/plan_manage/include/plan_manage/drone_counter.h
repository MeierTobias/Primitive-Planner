#ifndef _DRONE_COUNTER_H_
#define _DRONE_COUNTER_H_

#include <quadrotor_msgs/CountDrones.h>
#include <std_msgs/Empty.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <memory>
#include <unordered_set>

namespace primitive_planner
{

enum AlgoPhase : int8_t
{
  NEIGHBOUR_ADVERTISEMENT,
  FIND_NEIGHBOURS_REQUEST,
  FIND_NEIGHBOURS_ANSWER,
  UPDATE,
  DETACH_CHILD,
  ATTACH_CHILD,
  COUNT_NODES_REQUEST,
  COUNT_NODES_ANSWER,
};

enum Recipient : int8_t
{
  EVERYONE = -2,
  MY_CHILDREN = -1,
  I_HAVE_NO_NEIGHBOURS = -1,
};

class DroneCounter;

struct Phase
{
  DroneCounter &node;
  Phase(DroneCounter &node)
    : node(node) {};

  virtual void countMessageCallback(const quadrotor_msgs::CountDrones &msg) = 0;
};

struct FindNeighbours;
struct DecideOnBranch;

class DroneCounter
{
public:
  void init(ros::NodeHandle &nh, const Eigen::Vector3d &position);

  int countDrones();

  void sendMessage(AlgoPhase type, int recipient, int payload);
  void add_child(int child_id);
  void remove_child(int child_id);
  template <class PhaseType>
  void set_phase();
  void unset_phase()
  {
    phase = nullptr;
  }

  int uuid;

  int parentUuid;
  int rootUuid;
  std::unordered_set<int> children;

  int min_neighbour;
  int min_neighbour_root;

  int drone_total;

private:
  const Eigen::Vector3d *position;

  std::unique_ptr<Phase> phase = nullptr;

  ros::Publisher count_pub_;
  ros::Subscriber count_sub_;
  ros::Subscriber debug_sub_;

  void countMessageCallback(const quadrotor_msgs::CountDrones &msg);
  void debugMessageCallback(const std_msgs::Empty &msg);

public:
  typedef std::unique_ptr<DroneCounter> Ptr;
};

} // namespace primitive_planner

#endif
