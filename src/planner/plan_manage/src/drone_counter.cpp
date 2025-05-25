#include "plan_manage/drone_counter.h"
#include <std_msgs/Empty.h>
#include <quadrotor_msgs/CountDrones.h>
#include <ros/callback_queue.h>
#include <ros/assert.h>
#include <ros/console.h>
#include <ros/console_backend.h>
#include <ros/init.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <cassert>
#include <sstream>

namespace primitive_planner
{

void DroneCounter::init(ros::NodeHandle &nh, int id, const Eigen::Vector3d &position)
{
  this->position = &position;
  ROS_DEBUG("[COUNT] init");
  count_pub_ = nh.advertise<quadrotor_msgs::CountDrones>("/distributed_count", 100);
  count_sub_ = nh.subscribe<const quadrotor_msgs::CountDrones &>("/distributed_count", 100, &DroneCounter::countMessageCallback, this);
  debug_sub_ = nh.subscribe<const std_msgs::Empty &>("/debug_count", 100, &DroneCounter::debugMessageCallback, this);

  uuid = id;
  ROS_ASSERT(uuid != Recipient::EVERYONE and uuid != Recipient::MY_CHILDREN);
  rootUuid = uuid;
  parentUuid = uuid;
  children.clear();
}

constexpr double MAX_RECV_RADIUS = 5;

void DroneCounter::countMessageCallback(const quadrotor_msgs::CountDrones &msg)
{
  Eigen::Vector3d sender_position(msg.position.x, msg.position.y, msg.position.z);

  if ((sender_position - *position).norm() > MAX_RECV_RADIUS)
    return;
  if ((sender_position - *position).norm() < 1e-6) // probably a message from myself
    return;

  if (msg.rootUuid < min_neighbour_root || min_neighbour_root == I_HAVE_NO_NEIGHBOURS)
  {
    min_neighbour = msg.sender;
    min_neighbour_root = msg.rootUuid;
  }

  if (msg.type == DETACH_CHILD && msg.recipient == uuid)
  {
    remove_child(msg.sender);
  }

  if (msg.type == ATTACH_CHILD && msg.recipient == uuid)
  {
    add_child(msg.sender);
  }

  if (msg.type == UPDATE && msg.sender == min_neighbour)
  {
    if (msg.rootUuid == rootUuid)
    {
      min_neighbour = I_HAVE_NO_NEIGHBOURS;
      min_neighbour_root = I_HAVE_NO_NEIGHBOURS;
    }
    else
    {
      min_neighbour_root = msg.rootUuid;
    }
  }

  if (phase)
  {
    phase->countMessageCallback(msg);
  }
}

void DroneCounter::sendMessage(AlgoPhase type, int recipient, int payload)
{
  quadrotor_msgs::CountDrones message;
  message.sender = uuid;
  message.rootUuid = rootUuid;

  message.type = type;
  message.recipient = recipient;
  message.payload = payload;
  message.position.x = (*position)(0);
  message.position.y = (*position)(1);
  message.position.z = (*position)(2);
  count_pub_.publish(message);
}

template <class PhaseType>
void DroneCounter::set_phase()
{
  phase = std::make_unique<PhaseType>(*this);
}

struct NonRoot : Phase
{
  NonRoot(DroneCounter &dc)
    : Phase(dc)
  {
    dc.parentUuid = dc.min_neighbour;
    dc.rootUuid = dc.min_neighbour_root;
    dc.min_neighbour = I_HAVE_NO_NEIGHBOURS;
    dc.min_neighbour_root = I_HAVE_NO_NEIGHBOURS;
    dc.sendMessage(ATTACH_CHILD, dc.min_neighbour, 0);
  };

  enum
  {
    NOT_WAITING,
    NEIGHBOURS,
    COUNT_DRONES,
  } waiting_for;
  unsigned int nb_children_answered = 0;

  void countMessageCallback(const quadrotor_msgs::CountDrones &msg) override
  {
    if (msg.type == FIND_NEIGHBOURS_REQUEST and msg.sender == node.parentUuid)
    {
      assert(msg.recipient == MY_CHILDREN);
      node.rootUuid = msg.rootUuid;

      node.sendMessage(FIND_NEIGHBOURS_REQUEST, MY_CHILDREN, 0);
      waiting_for = NEIGHBOURS;
      nb_children_answered = 0;
    }
    else if (msg.type == FIND_NEIGHBOURS_ANSWER and msg.recipient == node.uuid)
    {
      ROS_ASSERT(waiting_for == NEIGHBOURS);

      nb_children_answered++;
      if (msg.payload < node.min_neighbour_root)
      {
        node.min_neighbour_root = msg.payload;
        node.min_neighbour = msg.sender;
      }
    }
    else if (msg.type == COUNT_NODES_REQUEST and msg.sender == node.parentUuid)
    {
      node.sendMessage(COUNT_NODES_REQUEST, MY_CHILDREN, 0);
      waiting_for = COUNT_DRONES;
    }
    else if (msg.type == COUNT_NODES_ANSWER)
    {
      if (msg.recipient == node.uuid)
      {
        assert(waiting_for == COUNT_DRONES);
        nb_children_answered++;
      }
      else if (msg.sender == node.parentUuid && msg.recipient == MY_CHILDREN)
      {
        node.drone_total = msg.payload;
        node.unset_phase();
        return;
      }
    }

    // This could happen after
    // - a DETACH_CHILD
    // - a new answer
    // - directly after the FIND_NEIGHBOURS_REQUEST, if I don't have children
    if (nb_children_answered == node.children.size())
    {
      if (waiting_for == NEIGHBOURS)
      {
        node.sendMessage(FIND_NEIGHBOURS_ANSWER, node.parentUuid, node.min_neighbour);
      }
      else if (waiting_for == COUNT_DRONES)
      {
        node.sendMessage(COUNT_NODES_ANSWER, node.parentUuid, node.drone_total);
      }

      waiting_for = NOT_WAITING;
    }

    if (msg.type == UPDATE && msg.payload == node.uuid)
    {
      node.add_child(msg.sender);
    }

    if (msg.type == UPDATE and msg.sender == node.parentUuid)
    {
      if (node.rootUuid == msg.rootUuid)
        return;
      node.rootUuid = msg.rootUuid;

      if (msg.payload == node.uuid)
      {
        node.parentUuid = node.min_neighbour;
      }

      node.sendMessage(UPDATE, MY_CHILDREN, node.parentUuid);
      return;
    }
  }
};

struct CountDrones : Phase
{
  int nb_children_answered = 0;

  CountDrones(DroneCounter &dc)
    : Phase(dc)
  {
    assert(node.rootUuid = node.uuid);
    assert(node.children.size() != 0);
    node.sendMessage(COUNT_NODES_REQUEST, MY_CHILDREN, 0);
  }

  void countMessageCallback(const quadrotor_msgs::CountDrones &msg) override
  {
    if (msg.type == COUNT_NODES_ANSWER)
    {
      assert(msg.recipient == node.uuid);
      nb_children_answered++;
      node.drone_total += msg.payload;
    }
    if (nb_children_answered == node.children.size())
    {
      node.unset_phase();
      return;
    }
  }
};

struct SearchForMinNeighbour : Phase
{
  unsigned short int nbDronesAnswered = 0;

  SearchForMinNeighbour(DroneCounter &dc)
    : Phase(dc)
  {
    node.sendMessage(FIND_NEIGHBOURS_REQUEST, MY_CHILDREN, 0);
    assert(node.rootUuid == node.uuid);
  }

  void countMessageCallback(const quadrotor_msgs::CountDrones &msg) override
  {
    if (msg.type != FIND_NEIGHBOURS_ANSWER or msg.recipient != node.uuid)
      return;

    nbDronesAnswered++;
    if (msg.payload < node.min_neighbour_root)
    {
      node.min_neighbour_root = msg.payload;
      node.min_neighbour = msg.sender;
    }
    if (nbDronesAnswered == node.children.size())
    {
      if (node.min_neighbour_root == I_HAVE_NO_NEIGHBOURS)
      {
        /* Search is finished, no new neigbours -> now count the nodes in the tree */
        node.set_phase<CountDrones>();
        return;
      }
      else if (node.min_neighbour_root < node.rootUuid)
      {
        node.parentUuid = node.min_neighbour;
        node.rootUuid = node.min_neighbour_root;
        node.sendMessage(UPDATE, MY_CHILDREN, node.parentUuid);

        node.set_phase<NonRoot>();
        return;
      }
      else if (node.min_neighbour_root > node.rootUuid)
      {
        node.set_phase<SearchForMinNeighbour>();
        return;
      }
      else
      {
        ROS_ERROR("[COUNT]");
      }
    }
  }
};

int DroneCounter::countDrones()
{
  min_neighbour = I_HAVE_NO_NEIGHBOURS;
  min_neighbour_root = I_HAVE_NO_NEIGHBOURS;
  sendMessage(NEIGHBOUR_ADVERTISEMENT, EVERYONE, 0);

  // Spin for 5 secs
  ros::Rate r(10); // 10 hz
  for (int i = 0; i < 70; i++)
  {
    ros::spinOnce();
    r.sleep();
  }

  /* We've received all neigbours ads we will get */
  if (min_neighbour == I_HAVE_NO_NEIGHBOURS)
  {
    return 1;
  }

  if (min_neighbour > uuid)
  { // We're still root
    set_phase<SearchForMinNeighbour>();
  }
  else
  {
    set_phase<NonRoot>();
  }
  while (phase)
  {
    ros::spinOnce();
  }
  return drone_total;
}

void DroneCounter::debugMessageCallback(const std_msgs::Empty &msg)
{
  int total = countDrones();
  std::stringstream ss;
  ss << "[COUNT]:\n"
     << "- uuid:" << uuid << "\n"
     << "- rootUuid:" << rootUuid << "\n"
     << "- total" << total << "\n";
  ROS_INFO(ss.str().c_str());
}

void DroneCounter::add_child(int child_id)
{
  ROS_ASSERT(children.find(child_id) == children.end());
  children.emplace(child_id);
}

void DroneCounter::remove_child(int child_id)
{
  std::size_t elements_removed = children.erase(child_id);
  ROS_ASSERT(elements_removed == 1);
}

} // namespace primitive_planner
