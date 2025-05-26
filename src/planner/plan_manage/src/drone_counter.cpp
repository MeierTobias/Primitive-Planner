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
  ROS_ASSERT(uuid != EVERYONE);
}

constexpr double MAX_RECV_RADIUS = 5;

void DroneCounter::startCounting()
{
  drone_total = 1;
  is_counting_nodes = true;
  nb_neighbours_answers_missing = 0;
  neighbours_acknowledged.clear();
  sendMessage(COUNT_NODES_REQUEST, parentUuid, rootUuid);
}

void DroneCounter::countMessageCallback(const quadrotor_msgs::CountDrones &msg)
{
  if (is_finished)
    return;

  Eigen::Vector3d sender_position(msg.position.x, msg.position.y, msg.position.z);

  if ((sender_position - *position).norm() > MAX_RECV_RADIUS)
    return;
  if ((sender_position - *position).norm() < 1e-6) // probably a message from myself
    return;

  // This is a no-op if the sender is already a known neighbour
  neighbours.insert(msg.sender);

  if (msg.type == FLOOD_FORWARD)
  {
    if (msg.payload < rootUuid)
    {
      rootUuid = msg.payload;
      parentUuid = msg.sender;
      is_counting_nodes = false;
      neighbours_acknowledged.clear();
      sendMessage(FLOOD_FORWARD, EVERYONE, rootUuid);
    }
  }
  else if (msg.type == COUNT_NODES_REQUEST)
  {
    if (msg.payload != rootUuid)
    {
      ROS_ASSERT(msg.payload >= rootUuid); // We should learn about the root from FLOOD message, not from COUNT_NODES_REQUEST
    }
    else
    {
      if (msg.sender == parentUuid)
      {
        startCounting();
      }
      else
      {
        neighbours_acknowledged.insert(msg.sender);
        if (msg.recipient == uuid)
        {
          nb_neighbours_answers_missing += 1;
        }
      }
    }
  }
  else if (msg.type == COUNT_NODES_ANSWER)
  {
    if (msg.sender == parentUuid)
    {
      ROS_ASSERT(!is_counting_nodes);
    }
    else
    {
      if (!is_counting_nodes)
        return;

      if (msg.recipient == uuid)
      {
        drone_total += msg.payload;
        nb_neighbours_answers_missing -= 1;
      }
    }
  }
  else if (msg.type == COUNT_NODES_FLOOD)
  {
    drone_total = msg.payload;
    sendMessage(COUNT_NODES_FLOOD, EVERYONE, msg.payload);
    is_finished = true;
  }
  else
  {
    ROS_ASSERT(false); // Unknown message type
  }

  // This can happen after a new acknowledgement (COUNT_NODES_REQUEST) or a new answer (COUNT_NODES_ANSWER)
  if (is_counting_nodes)
  {
    ROS_DEBUG("[COUNT] node %d waiting...; msg_type=%d, %d/%d neighbours known, still waiting for %d answers", uuid, msg.type, neighbours_acknowledged.size(), neighbours.size(), nb_neighbours_answers_missing);
    if (nb_neighbours_answers_missing == 0)
    {
      if (rootUuid == uuid)
      {
        if (neighbours_acknowledged.size() == neighbours.size())
        {
          // The root receives answers from all of its children
          sendMessage(COUNT_NODES_FLOOD, EVERYONE, drone_total);
          is_finished = true;
          is_counting_nodes = false;
        }
      }
      else
      {
        if (neighbours_acknowledged.size() == neighbours.size() - 1)
        {
          // Other nodes do not receive an answer from their parent
          sendMessage(COUNT_NODES_ANSWER, parentUuid, drone_total);
          is_counting_nodes = false;
        }
      }
    }
  }
}

void DroneCounter::sendMessage(AlgoPhase type, int recipient, int payload)
{
  quadrotor_msgs::CountDrones message;
  message.sender = uuid;

  if (type == FLOOD_FORWARD)
    ROS_ASSERT(recipient == EVERYONE);
  else if (type == COUNT_NODES_REQUEST)
    ROS_ASSERT(recipient == parentUuid);
  else if (type == COUNT_NODES_ANSWER)
    ROS_ASSERT(recipient == parentUuid);
  else if (type == COUNT_NODES_FLOOD)
    ROS_ASSERT(recipient == EVERYONE);

  message.type = type;
  message.recipient = recipient;
  message.payload = payload;
  message.position.x = (*position)(0);
  message.position.y = (*position)(1);
  message.position.z = (*position)(2);
  count_pub_.publish(message);
}

int DroneCounter::countDrones()
{
  rootUuid = uuid;
  parentUuid = uuid;
  is_finished = false;
  is_counting_nodes = false;
  neighbours.clear();

  // Spin for 5 secs to make sure everyone has reset
  ros::Rate r(10); // 10 hz
  for (int i = 0; i < 70; i++)
  {
    ros::spinOnce();
    r.sleep();
  }

  sendMessage(FLOOD_FORWARD, EVERYONE, uuid);

  // Spin for 5 secs
  for (int i = 0; i < 70; i++)
  {
    ros::spinOnce();
    r.sleep();
  }

  /* We've received all neigbours ads we will get */
  if (neighbours.empty())
  {
    return 1;
  }

  if (rootUuid == uuid)
  { // We're still root
    startCounting();
  }
  while (!is_finished)
  {
    ros::spinOnce();
    r.sleep();
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

} // namespace primitive_planner
