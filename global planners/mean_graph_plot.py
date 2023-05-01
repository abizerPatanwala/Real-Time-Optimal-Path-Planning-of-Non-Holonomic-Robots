"""
Mean Graph Plot
"""

import math
import numpy as np
import os
import sys

import matplotlib.pyplot as plt

bit_star_mean_cost = [33.8, 29.94, 37.92, 38.54, 29.68, 50, 50, 38.2, 45.5, 41.2, 37.92, 38.54, 29.68, 50, 38.2, 45.5, 29.94, 37.92, 38.54, 29.68]
ait_star_mean_cost = [32.8, 33, 32.5, 40, 35.1, 50, 45.26, 68.33, 43.33, 43.2, 32.5, 40, 35.1, 50, 35.1, 50, 45.26, 68.33, 40, 35.1]
rrt_mean_cost = [75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75,75]

runs = np.linspace(1,20, 20)
xticks = range(len(runs))
print(runs)

plt.figure()
plt.title('Mean Cost')
plt.xlabel('Trials')
plt.ylabel('Mean Cost')
plt.plot(runs, bit_star_mean_cost, label = "BIT*")
plt.plot(runs, ait_star_mean_cost, label = "AIT*")
plt.plot(runs, rrt_mean_cost, label = "RRT")
plt.xticks(xticks, runs)
plt.legend()
plt.show()

bit_star_total_nodes = [175, 200, 125, 260, 180, 420, 500, 400, 600, 375, 260, 180, 500, 400, 375, 200, 125, 175, 260, 180]
ait_star_total_nodes = [140, 160, 120, 200, 160, 420, 380, 520, 400, 325, 340, 155, 210, 425, 350, 170, 185, 300, 440, 160]
rrt_total_nodes = [1600, 500, 450, 1750, 2000, 2000, 1100, 700, 650, 700, 500, 1600, 1900, 1560, 240, 1500, 750, 960, 740, 800]

plt.figure()
plt.title('Total No. of Nodes Explored')
plt.xlabel('Trials')
plt.ylabel('No. of Nodes')
plt.plot(runs, bit_star_total_nodes, label = "BIT*")
plt.plot(runs, ait_star_total_nodes, label = "AIT*")
plt.plot(runs, rrt_total_nodes, label = "RRT")
plt.xticks(xticks, runs)
plt.legend()
plt.show()
