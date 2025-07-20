# coding: utf-8
import matplotlib.pyplot as plt
from glob import glob
files = glob('data/clearance*.v')
x, y = [], []
for file in files:
    with open(file, 'r') as file:
        try:
            lastline = file.readlines()[-1]
        except IndexError:
            continue
    clearance, radius = lastline.split(': ')
    clearance = float(clearance)
    radius = float(radius)
    x.append(clearance)
    y.append(radius)
fig, ax = plt.subplots()
ax.scatter(x, y)
ax.set_ylabel('Minimum goal radius [m]')
ax.set_xlabel('Swarm clearance [m]')
plt.show()
