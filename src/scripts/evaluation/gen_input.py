import time
import argparse
import rosbag
from geometry_msgs.msg import Twist
from rospy.rostime import Time, Duration


def make_periodic_U_turn_bag(bag_path, hz=1.0, one_way_duration=25.0, n_turns=10):
    """
    Generates a bag at `bag_path` containing a simple Twist command
    published at `hz` that lets the drones fly in one direction for
    'one_way_duration' seconds and then in the opposite direction.
    """
    bag = rosbag.Bag(bag_path, "w")
    try:
        period = 1.0 / hz
        t0 = Time(time.time())
        dir = 1
        steps_per_leg = int(one_way_duration * hz)
        for i in range(n_turns):
            for j in range(steps_per_leg):
                t = t0 + Duration.from_sec((i * steps_per_leg + j) * period)
                twist = Twist()
                twist.linear.x = 0.5 * dir

                bag.write("/cmd_vel", twist, t)
            dir *= -1
    finally:
        t = t0 + Duration.from_sec(
            ((n_turns - 1) * steps_per_leg + (steps_per_leg - 1) + 1) * period
        )
        twist = Twist()
        twist.linear.x = 0.0
        bag.write("/cmd_vel", twist, t)

        bag.close()
        print(f"Wrote {bag_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Generate a rosbag of /cmd_vel Twist messages."
    )
    parser.add_argument(
        "--bag_path",
        type=str,
        default="cmd_vel_u-turn.bag",
        help="Output bag filename (e.g. cmd_vel.bag)",
    )
    parser.add_argument(
        "--hz", type=float, default=1.0, help="Publish rate in Hz (default: 1.0)"
    )
    parser.add_argument(
        "--one_way_duration",
        type=float,
        default=25.0,
        help="Duration to fly in one direction (default: 25.0)",
    )
    parser.add_argument(
        "--n_turns", type=int, default=10, help="Number of U-turns (default: 10)"
    )
    args = parser.parse_args()

    make_periodic_U_turn_bag(
        bag_path=args.bag_path,
        hz=args.hz,
        one_way_duration=args.one_way_duration,
        n_turns=args.n_turns,
    )
