#!/usr/bin/env python3
import os
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":
    data_folder = os.path.join(os.path.dirname(__file__), "data")
    files = [f for f in os.listdir(data_folder)
             if f.startswith('u-turn_out_')
             and f.endswith('_post.npz')
             and os.path.isfile(os.path.join(data_folder, f))]

    fig, axs = plt.subplots(nrows=3, ncols=1)
    for file in files:
        data = np.load(os.path.join(data_folder, file))
        axs[0].plot(data['R_g'])
        axs[1].plot(data['D_avg'])
        axs[2].plot(data['D_max'])
        data.close()

    plt.show()
