#!/usr/bin/env python3
import os
import numpy as np
import matplotlib.pyplot as plt

if __name__ == "__main__":

    data = np.load(os.path.join(os.path.dirname(__file__), "results_u-turn.npz"))

    fig, axs = plt.subplots(nrows=3, ncols=1)
    axs[0].plot(data['R_g'])
    axs[1].plot(data['D_avg'])
    axs[2].plot(data['D_max'])
    plt.show()

    data.close()