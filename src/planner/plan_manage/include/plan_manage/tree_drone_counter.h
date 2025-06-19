#ifndef _TREE_DRONE_COUNTER_H_
#define _TREE_DRONE_COUNTER_H_

#include <primitive_planner/TreeCountDrones.h>
#include "drone_counter.h"
#include <std_msgs/Empty.h>
#include <ros/subscriber.h>
#include <ros/node_handle.h>
#include <ros/ros.h>
#include <Eigen/Core>
#include <memory>
#include <unordered_set>
#include <cstdint>

namespace primitive_planner
{

enum AlgoPhase : int8_t
{
  // We're doing a flood-fill from the root, i.e. the node with the lowest ID.
  // We don't know who the root is, so everyone starts flooding and the flood from the root overwrites everything else.
  FLOOD_FORWARD,
  // Then we're flooding, starting at the root, requesting a count of drones.
  // For this, we set the recipient to the parent (so they can listen and take this as an acknowledgement that they're our parent)
  // and set the payload to the root (to make sure that it doesn't interfere if any other roots started a request)
  COUNT_NODES_REQUEST,
  // After the request, every node knows what its children are.
  // Once all children have answered (i.e. immediately if we had no children), we propagate the answer back up the tree.
  COUNT_NODES_ANSWER,
  // When the root receives all answers, it knows the number of nodes. It then passes that number back down the tree.
  COUNT_NODES_FLOOD,
};

constexpr int8_t EVERYONE = -1; // Default recipient.
// It doesn't really matter, all message types have either a set recipient (COUNT_NODES_ANSWER, COUNT_NODES_REQUEST)
// or no recipient and are implicitly addressed to all children (i.e. EVERYONE)

class TreeDroneCounter : public DroneCounter
{
public:
  void init(ros::NodeHandle &nh, int id, const Eigen::Vector3d &position);

  int countDrones();

  int uuid;

  int parentUuid;
  int rootUuid;
  std::unordered_set<int> neighbours;
  std::unordered_set<int> neighbours_acknowledged;
  unsigned int nb_neighbours_answers_missing;

  bool is_counting_nodes;
  bool is_finished = true;

private:
  bool isRoot() const
  {
    return rootUuid == uuid;
  }
  void startCounting();
  void sendMessage(AlgoPhase type, int recipient = EVERYONE, int payload = 0);

  void countMessageCallback(const primitive_planner::TreeCountDrones &msg);

public:
  typedef std::unique_ptr<TreeDroneCounter> Ptr;
};

} // namespace primitive_planner

#endif
