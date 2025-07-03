#!/usr/bin/env python3
import time
import rosbag
from quadrotor_msgs.msg import GoalSet
from rospy.rostime import Time, Duration


def make_goal_point_bag(bag_path, point_list, duration_list):

    bag = rosbag.Bag(bag_path, "w")

    t = Time(time.time())
    for point, duration in zip(point_list, duration_list):
        t += Duration.from_sec(duration)
        pose = GoalSet()
        pose.drone_id = 0
        pose.goal = point

        bag.write("/goal_with_id", pose, t)

    bag.close()
    print(f"Wrote {bag_path}")

if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('out_path')
    args = parser.parse_args()

    points = [ (10.0, 0.0, 0.0), (0.0, -10.0, 0.0), (-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), (0.0, 0.0, 0.0)]
    durations = [ 20 for _ in range(len(points)) ]

    make_goal_point_bag(args.out_path, points, durations)
