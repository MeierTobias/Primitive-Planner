#!/usr/bin/env python3
import os
import rosbag
import numpy as np
from geometry_msgs.msg import PoseWithCovariance, Pose, Point
from nav_msgs.msg import Odometry
import matplotlib.pyplot as plt


def load_odom(bag, topic):
    """
    Read nav_msgs/Odometry messages from `topic` into numpy arrays:
      t: shape (N,), x,y,z: (N,), vx,vy,vz: (N,)
    """
    ts, xs, ys, zs, vxs, vys, vzs = [], [], [], [], [], [], []
    for _, msg, t in bag.read_messages(topics=[topic]):
        ts.append(t.to_sec())
        p = msg.pose.pose.position
        xs.append(p.x)
        ys.append(p.y)
        zs.append(p.z)
        v = msg.twist.twist.linear
        vxs.append(v.x)
        vys.append(v.y)
        vzs.append(v.z)

    if not ts:
        return None

    # convert to numpy arrays
    ts = np.array(ts)

    xs = np.array(xs)
    ys = np.array(ys)
    zs = np.array(zs)
    vxs = np.array(vxs)
    vys = np.array(vys)
    vzs = np.array(vzs)

    # sort by time (just in case)
    order = np.argsort(ts)
    return {
        "ts": ts[order],
        "position": np.vstack((xs[order], ys[order], zs[order])),
        "velocity": np.vstack((vxs[order], vys[order], vzs[order])),
    }


def aggregate_odom(data, dt=0.1):
    """
    Aggregate odometry so you have one sample every `dt` seconds.

    Input:
      data: dict with keys
        - "ts"       : 1D array of times, shape (N,)
        - "position" : 2D array, shape (3, N)
        - "velocity" : 2D array, shape (3, N)
      dt: bin width in seconds (default 0.1)

    Returns a dict with the same keys, but:
      - "ts"       : 1D array of bin‐center times, shape (M,)
      - "position" : 2D array, shape (3, M) of mean positions per bin
      - "velocity" : 2D array, shape (3, M) of mean velocities per bin
    """
    ts = data["ts"]
    pos = data["position"]
    vel = data["velocity"]

    # define bins from t0 up to the last sample
    t0, t_last = ts[0], ts[-1]
    # bin edges: [t0, t0+dt, t0+2dt, ..., >=t_last]
    edges = np.arange(t0, t_last + dt, dt)
    # assign each sample to a bin index
    # bins[i] = index of the bin (1..len(edges)-1) into which ts[i] falls
    bins = np.digitize(ts, edges) - 1

    M = len(edges) - 1
    agg_ts = edges[:-1] + dt / 2  # bin centers
    agg_pos = np.zeros((3, M))
    agg_vel = np.zeros((3, M))

    for b in range(M):
        mask = bins == b
        if not np.any(mask):
            # no samples in this bin → set to NaN (or you could carry-last-value)
            agg_pos[:, b] = np.nan
            agg_vel[:, b] = np.nan
        else:
            agg_pos[:, b] = pos[:, mask].mean(axis=1)
            agg_vel[:, b] = vel[:, mask].mean(axis=1)

    return {"ts": agg_ts, "position": agg_pos, "velocity": agg_vel}


def radius_of_gyration(positions):
    """
    positions: np.ndarray of shape (d, T, N)
      d = dimension,
      T = time steps,
      N = number of drones
    Returns:
      Rg: np.ndarray of shape (T,), the RMS radius at each time t
    """
    # compute centroid at each t: shape (d, T)
    centroid = positions.mean(axis=2)
    # displacement from centroid: shape (d, T, N)
    disp = positions - centroid[:, :, None]
    # squared distances per drone: sum over d → shape (T, N)
    sqd = np.sum(disp**2, axis=0)
    # mean over N and sqrt → shape (T,)
    Rg = np.sqrt(sqd.mean(axis=1))
    return Rg


def pairwise_distance(positions):
    """
    positions: np.ndarray of shape (d, T, N)
    Returns:
      Davg: np.ndarray of shape (T,), the mean pairwise distance at each t
      Dmax: np.ndarray of shape (T,), the max pairwise distance at each t
    """
    d, T, N = positions.shape
    Davg = np.zeros(T)
    Dmax = np.zeros(T)
    for t in range(T):
        # per‐time slice: (d, N) → transpose to (N, d)
        X = positions[:, t, :].T
        # all pairwise differences: (N, N, d)
        diffs = X[:, None, :] - X[None, :, :]
        # Euclidean distances: (N, N)
        dists = np.linalg.norm(diffs, axis=2)
        # average over i<j
        i, j = np.triu_indices(N, k=1)
        Davg[t] = dists[i, j].mean()
        Dmax[t] = dists[i, j].max()
    return Davg, Dmax


if __name__ == "__main__":

    output_bag_file = os.path.join(os.path.dirname(__file__), "output_u-turn.bag")

    bag = rosbag.Bag(output_bag_file)
    info = bag.get_type_and_topic_info().topics

    data = {}
    for t, stat in info.items():
        data[t] = aggregate_odom(load_odom(bag, t), dt=0.1)

    series_length = np.min([topic["position"].shape[1] for topic in data.values()])
    topic_length = len(data.keys())
    position_tensor = np.zeros((3, series_length, topic_length))

    for i, topic in enumerate(data.values()):
        position_tensor[:, :, i] = topic["position"][:, 0:series_length]

    R_g = radius_of_gyration(position_tensor)

    D_avg, D_max = pairwise_distance(position_tensor)

    np.savez(os.path.join(os.path.dirname(__file__), "results_u-turn.npz"), R_g=R_g,D_avg=D_avg, D_max=D_max)

    bag.close()
