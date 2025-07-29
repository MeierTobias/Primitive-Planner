# coding: utf-8
from collections import defaultdict
import sys
import math
import matplotlib
import matplotlib.pyplot as plt
import seaborn as sns
from scipy.optimize import curve_fit

sns.set_theme(style="darkgrid", palette="flare")
matplotlib.rcParams['mathtext.fontset'] = 'stix'
matplotlib.rcParams['font.family'] = 'STIXGeneral'

files = sys.argv[1:]
data = defaultdict(list)
for file in files:
    with open(file, 'r') as file:
        try:
            lastline = file.readlines()[-1]
        except IndexError:
            continue
    clearance, radius = lastline.split(': ')
    clearance = float(clearance)
    radius = float(radius)
    data[clearance].append(radius)
fig, ax = plt.subplots()

x = list(data.keys())
def mean(li):
    return sum(li) / len(li)
y = [mean(li) for li in data.values()]
ax.scatter(x, y)
ax.set_ylabel('Minimum goal radius [m]')
ax.set_xlabel('Swarm clearance [m]')

densest_packing_20 = 47.4310362
coeff = 1/math.sin(math.radians(densest_packing_20/2))
ax.axline((0, 0), slope=coeff, color="black", linestyle=(0, (5,5)), label="Theoretical densest packing")
print("\"Theoretical densest packing\":", coeff)

p_fit, _ = curve_fit(
    lambda x, p: p * x,
    x, y
)
ax.axline((0, 0), slope=p_fit[0], linestyle="dotted", label=f"Best fit: $p={p_fit[0]:.2f}$")
print("\"Best fit\":", p_fit)

ax.legend()
plt.show()
