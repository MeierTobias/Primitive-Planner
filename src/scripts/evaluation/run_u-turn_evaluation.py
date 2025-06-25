#!/usr/bin/env python3
import os
import subprocess
import time
import sys
import signal
import argparse

from gen_input import make_periodic_U_turn_bag

if __name__ == "__main__":

    input_bag_file = os.path.join(os.path.dirname(__file__), "cmd_vel_u-turn.bag")

    # check if the input bag file exists
    if not os.path.exists(input_bag_file):
        print(f"Input bag '{input_bag_file}' was not found. Generating it now...")
        make_periodic_U_turn_bag(
            bag_path=input_bag_file,
            hz=1.0,
            one_way_duration=25.0,
            n_turns=10,
        )

    # launch the drone nodes
    sim = subprocess.Popen(["roslaunch", "primitive_planner", "teleop_heading.launch"])
    time.sleep(10)

    # start recording all topics
    record_cmd = ["rosbag", "record", "-O", "output.bag"]
    for i in range(5):
        record_cmd += [f"/drone_{i}_visual_slam/odom"]

    recorder = subprocess.Popen(record_cmd)

    # play the cmd_vel bag
    subprocess.run(["rosbag", "play", input_bag_file, "--clock"], check=True)

    # clean up
    recorder.send_signal(signal.SIGINT)
    recorder.wait()
    sim.terminate()
    sim.wait()
