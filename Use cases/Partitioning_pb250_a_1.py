"""Partitioning clients' list for pb250_a - A separate study"""

import pyDroneDeliv.processing as pro
import pyDroneDeliv.pre_processing as pre
import pyDroneDeliv.post_processing as post
import matplotlib.pyplot as plt
# import numpy as np

drone1 = pre.Drone(180, 10.3, 0.013)
drone2 = pre.Drone(510, 12.5, 0.024)

wind1 = pre.Wind(0, 0)

problem1 = pre.Problem()
problem1.import_csv('pb250_b.csv')

param1 = pre.DeliveryParameters(drone2, wind1, pro.cost_b)

pro.clarke_and_wright(problem1, param1, version="parallel")

post.plot_problem_solutions(problem1)
plt.show()
