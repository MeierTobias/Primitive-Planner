#!/usr/bin/env python3
import time
import rosbag
from quadrotor_msgs.msg import GoalSet
from rospy.rostime import Time, Duration
import math


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
    parser.add_argument('-o', '--out-path')
    parser.add_argument('--out-yaml-starting-positions', action='store_true')
    parser.add_argument('-s', '--scenario', type=int)
    args = parser.parse_args()

    scenarios = [
        (
            [(10.0, 0.0, 0.0), (0.0, -10.0, 0.0), (-10.0, 0.0, 0.0), (-10.0, 0.0, 0.0), (0.0, 0.0, 0.0)],
            [20 for _ in range(5)],
        ),
        (
            [(20 * math.cos(2 * math.pi * theta / 12), 20 * math.sin(2*math.pi*theta/12), 0.0) for theta in range(12)],
            [10 for _ in range(10)],
        )
    ]

    if args.out_yaml_starting_positions:
        num_drones = 20
        positions = [
            [(i, -8.0) for i in range(num_drones)],
            [(math.cos(2*math.pi*theta/num_drones), math.sin(2*math.pi*theta/num_drones)) for theta in range(num_drones)],
        ]
        print("starting_positions:")
        for (x, y) in positions[args.scenario]:
            print("-")
            print(f"  - {x:.3f}")
            print(f"  - {y:.3f}")
    else:
        points, durations = scenarios[args.scenario]
        make_goal_point_bag(args.out_path, points, durations)
