#!/usr/bin/env python3
import itertools
import os
import subprocess
import time
import signal
import rosbag

import numpy as np

from geometry_msgs.msg import PoseWithCovariance, Pose, Point
from nav_msgs.msg import Odometry

from gen_input import make_periodic_U_turn_bag
from processing import preprocess_data, radius_of_gyration, pairwise_distance


def grid_convex_weights(step=0.125, w_min=0.125):
    """
    Return an array of shape (M,4) containing every (w1,w2,w3,w4)
    such that each wi in [w_min, 1] in increments of `step`
    and sum(w) == 1.
    """
    # max any single wi can be, given the other 3 are at least w_min
    w_max = 1 - 3 * w_min
    # 1-D array of possible wi values
    vals = np.arange(w_min, w_max + 1e-9, step)

    # Cartesian product of 4 picks, then filter by sum==1
    weights = [w for w in itertools.product(vals, repeat=4) if abs(sum(w) - 1.0) < 1e-6]

    return np.array(weights)


if __name__ == "__main__":

    drone_count = 5

    input_bag_file = os.path.join(
        os.path.dirname(__file__), "data", "u-turn_cmd_vel.bag"
    )

    # check if the input bag file exists
    if not os.path.exists(input_bag_file):
        print(f"Input bag '{input_bag_file}' was not found. Generating it now...")
        make_periodic_U_turn_bag(
            bag_path=input_bag_file,
            hz=1.0,
            one_way_duration=25.0,
            n_turns=10,
        )

    # TODO: generate launch file

    # generate convex weight set
    weights = grid_convex_weights(step=0.1, w_min=0.1)

    # loop over weights
    for (
            lambda_d,
            lambda_heading_virtual,
            lambda_heading_neighbors,
            lambda_contraction,
    ) in weights:

        # construct the bag output path
        out_bag_path = os.path.join(
            os.path.dirname(__file__),
            "data",
            f"u-turn_out_{lambda_d:.3f}_{lambda_heading_virtual:.3f}_{lambda_heading_neighbors:.3f}_{lambda_contraction:.3f}.bag",
        )

        # check if the file already exists
        if not os.path.isfile(out_bag_path):

            print(
                f"running experiment (lambda_d={lambda_d:.3f} lambda_heading_virtual={lambda_heading_virtual:.3f} lambda_heading_neighbors={lambda_heading_neighbors:.3f} lambda_contraction={lambda_contraction:.3f})",
            )

            # launch the drone nodes
            launch_cmd = [
                "roslaunch",
                os.path.join(os.path.dirname(__file__), "u-turn.launch"),
            ]
            launch_cmd += ["lambda_l:=20.0"]
            launch_cmd += ["lambda_b:=5.0"]
            launch_cmd += [f"lambda_d:={lambda_d}"]
            launch_cmd += [f"lambda_heading_virtual:={lambda_heading_virtual}"]
            launch_cmd += [f"lambda_heading_neighbors:={lambda_heading_neighbors}"]
            launch_cmd += [f"lambda_contraction:={lambda_contraction}"]

            sim = subprocess.Popen(launch_cmd)
            time.sleep(10)

            # start recording all topics
            record_cmd = [
                "rosbag",
                "record",
                "-O",
                out_bag_path,
            ]
            for i in range(drone_count):
                record_cmd += [f"/drone_{i}_visual_slam/odom"]
            record_cmd += [f"/cmd_vel"]

            recorder = subprocess.Popen(record_cmd)

            # play the cmd_vel bag
            subprocess.run(["rosbag", "play", input_bag_file, "--clock"], check=True)

            # clean up
            recorder.send_signal(signal.SIGINT)
            recorder.wait()
            sim.terminate()
            sim.wait()

            time.sleep(2)

        else:
            print(
                f"experiment found (lambda_d={lambda_d:.3f} lambda_heading_virtual={lambda_heading_virtual:.3f} lambda_heading_neighbors={lambda_heading_neighbors:.3f} lambda_contraction={lambda_contraction:.3f})",
            )

        pre_processed_file = f"{out_bag_path[:-4]}_pre.npz"
        if not os.path.isfile(pre_processed_file):
            position_tensor = preprocess_data(out_bag_path, pre_processed_file)
        else:
            position_tensor = np.load(pre_processed_file)

        post_processed_file = f"{out_bag_path[:-4]}_post.npz"
        if not os.path.isfile(post_processed_file):
            R_g = radius_of_gyration(position_tensor)
            R_g_avg = np.mean(R_g)
            R_g_max = np.max(R_g)

            D_avg, D_max = pairwise_distance(position_tensor)
            D_avg_avg = np.mean(D_avg)
            D_avg_max = np.max(D_avg)
            D_max_avg = np.mean(D_max)
            D_max_max = np.max(D_max)

            np.savez(
                os.path.join(os.path.dirname(__file__), post_processed_file),
                R_g=R_g,
                D_avg=D_avg,
                D_max=D_max,
                R_g_avg=R_g_avg,
                R_g_max=R_g_max,
                D_avg_avg=D_avg_avg,
                D_avg_max=D_avg_max,
                D_max_avg=D_max_avg,
                D_max_max=D_max_max,
            )
