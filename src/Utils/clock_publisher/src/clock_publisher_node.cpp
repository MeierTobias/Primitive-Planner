#include <ros/ros.h>
#include <rosgraph_msgs/Clock.h>

int main(int argc, char** argv) {

  ros::init(argc, argv, "clock_publisher_node");
  ros::NodeHandle nh("~");

  double time_multiplier;
  nh.param("time_multiplier", time_multiplier, 0.1);
  ROS_INFO_STREAM("clock_publisher: time_multiplier = " << time_multiplier);

  ros::Publisher clock_pub = nh.advertise<rosgraph_msgs::Clock>("/clock", 10);
  ros::WallRate rate(10.0);

  double start_wall = ros::WallTime::now().toSec();

  while (ros::ok()) {
    double elapsed_wall = ros::WallTime::now().toSec() - start_wall;
    rosgraph_msgs::Clock clock_msg;
    clock_msg.clock = ros::Time( elapsed_wall * time_multiplier);

    clock_pub.publish(clock_msg);
    rate.sleep();
  }

  return 0;
}
