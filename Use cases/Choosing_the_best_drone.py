"""Case study : Choosing the best drone between 3 possible drones"""
import pyDroneDeliv.processing as pro
import pyDroneDeliv.pre_processing as pre
# import pyDroneDeliv.post_processing as post
# import matplotlib.pyplot as plt
import numpy as np

drone1 = pre.Drone(70, 7.7, 0.009)
drone2 = pre.Drone(200, 10.5, 0.012)
drone3 = pre.Drone(450,13.2, 0.018)

wind1 = pre.Wind(np.random.uniform(2.0, 3.5), 0)
# this time, we consider that wind goes from west to east, and its magnitude goes from 2.0 m/s to 3.5 m/s

depot1 = pre.Depot("Chatenay-Malabry", 0, 0)

param1 = pre.DeliveryParameters(drone1, wind1, pro.cost_b)

total_cost = 0
total_savings = 0
for i in range(500):
    print(i)
    problem_i = pre.Problem(depot1)
    problem_i.generate_random_clients(amount=np.random.randint(50, 60),
                                      x=(-6000, 6000), y=(-3500, 3500),
                                      demand=(5, 60))
    pro.clarke_and_wright(problem_i, param1, version="sequential")
    solution_i = problem_i.solutions_list[0]
    total_cost += solution_i.cost_and_savings()[0]
    total_savings += solution_i.cost_and_savings()[1]
average = total_savings / (total_savings + total_cost)
print("Drone1.", "La moyenne d'économie en séquentielle pour 1000 problèmes indépendants est {}".format(average))
