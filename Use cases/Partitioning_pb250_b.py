"""Case study - Partitioning clients' lists for pb250_b"""

import pyDroneDeliv.processing as pro
import pyDroneDeliv.pre_processing as pre
# import pyDroneDeliv.post_processing as post
# import matplotlib.pyplot as plt
# import numpy as np

drone1 = pre.Drone(180, 10.3, 0.013)
drone2 = pre.Drone(510, 12.5, 0.024)

wind1 = pre.Wind(0, 0)

param1 = pre.DeliveryParameters(drone1, wind1, pro.cost_b)
param2 = pre.DeliveryParameters(drone2, wind1, pro.cost_b)

problem_g = pre.Problem()
problem_g.import_csv('pb250_b.csv')

best_cost = float('inf')
limit = 0
for i in range(5, 151):
    clients_list_1 = []
    clients_list_2 = []
    for client in problem_g.clients_list:
        if client.demand > i:
            clients_list_2.append(client)
        else:
            clients_list_1.append(client)
    problem1 = pre.Problem(problem_g.depot, clients_list_1)
    problem2 = pre.Problem(problem_g.depot, clients_list_2)
    init_1 = pro.clarke_and_wright_init(problem1, param1)
    init_2 = pro.clarke_and_wright_init(problem2, param2)
    deliveries_1 = pro.parallel_build_deliveries(problem1, param1, *init_1)
    deliveries_2 = pro.parallel_build_deliveries(problem2, param2, *init_2)
    solution1 = pre.Solution("drone1", deliveries_1, param1)
    solution2 = pre.Solution("drone2", deliveries_2, param2)
    if solution1.cost_and_savings()[0] + solution2.cost_and_savings()[0] < best_cost:
        best_cost = solution1.cost_and_savings()[0] + solution2.cost_and_savings()[0]
        limit = i
    print(limit, best_cost)

# min = float('inf')
# max = 0
# for client in problem_g.clients_list:
#     if client.demand < min:
#         min = client.demand
#     if client.demand > max:
#         max = client.demand
# print(min)
# print(max)


# post.plot_problem_solutions(problem_g)
# plt.show()
