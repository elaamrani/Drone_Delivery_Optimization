"""Case study - Partitioning clients' lists"""

import pyDroneDeliv.pre_processing as pre
# import numpy as np
import pyDroneDeliv.processing as pro
import pyDroneDeliv.post_processing as post
import matplotlib.pyplot as plt

drone1 = pre.Drone(180, 10.3, 0.013)
drone2 = pre.Drone(510, 12.5, 0.024)
wind1 = pre.Wind(0, 0)

problem_g = pre.Problem()
problem_g.import_csv('pb250_b.csv')

clients_list_1 = []
clients_list_2 = []
for client in problem_g.clients_list:
    if client.x > 0 and client.y > 0:
        clients_list_1.append(client)
    if client.x < 0 and client.y < 0:
        clients_list_2.append(client)

problem1 = pre.Problem(problem_g.depot, clients_list_1)
problem2 = pre.Problem(problem_g.depot, clients_list_2)
param1 = pre.DeliveryParameters(drone1, wind1, pro.cost_b)
param2 = pre.DeliveryParameters(drone2, wind1, pro.cost_b)

pro.clarke_and_wright(problem1, param1, version="parallel", name="Drone1 - Partition")
pro.clarke_and_wright(problem2, param2, version="parallel", name="Drone2 - Partition")

init_1 = pro.clarke_and_wright_init(problem1, param1)
init_2 = pro.clarke_and_wright_init(problem2, param1)
deliveries_1 = pro.parallel_build_deliveries(problem1, param1, *init_1)
deliveries_2 = pro.parallel_build_deliveries(problem2, param2, *init_2)
solution1 = pre.Solution("drone1", deliveries_1, param1)
solution2 = pre.Solution("drone2", deliveries_2, param2)

problem_g.solutions_list = [solution1, solution2]

post.plot_problem_solutions(problem_g)
plt.show()

