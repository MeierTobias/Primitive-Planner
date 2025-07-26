#!/usr/bin/env python3
import time
import argparse
import rosbag
from geometry_msgs.msg import PoseStamped
from rospy.rostime import Time, Duration


def make_goal_point_bag(bag_path, point_list, duration_list):

    bag = rosbag.Bag(bag_path, "w")

    t = Time(time.time())
    for point, duration in zip(point_list, duration_list):
        t += Duration.from_sec(duration)
        pose = PoseStamped()
        pose.pose.position.x = point[0]
        pose.pose.position.y = point[1]
        pose.pose.position.x = point[2]

        bag.write("/goal", pose, t)

    bag.close()
    print(f"Wrote {bag_path}")
