# coding: utf-8
import subprocess
import rosbag

def passes(radius: float, clearance: float) -> bool:
    file = f"data/goal_radius_{radius}_{clearance}.bag"
    subprocess.run(["snakemake", "-c", "1", file], stdout=subprocess.DEVNULL)
    with rosbag.Bag(file, 'r') as bag:
        return (sum(1 for _ in bag) >= 3)


def bisect(*args, **kwargs) -> float:
    min_radius = 0.0
    max_radius = 20.0
    while max_radius - min_radius > 0.1:
        radius = (max_radius + min_radius) / 2
        if passes(radius, *args, **kwargs):
            print('<=', radius)
            max_radius = radius
        else:
            print(">=", radius)
            min_radius = radius
    return (min_radius + max_radius) / 2


if __name__ == "__main__":
    import sys

    clearance = float(sys.argv[1])
    print(f"{clearance}: {bisect(clearance)}")
