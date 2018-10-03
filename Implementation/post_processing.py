"""Implements a few functions to help visualise problems and theirs solutions."""
import numpy as np
import matplotlib.pyplot as plt  # very powerful module when it comes to plotting things


def place_holder(*args):
    """This function does nothing. It is just a place holder that is used to fill the 'holes' in the code. Its only
    purpose is to make the code look nice (syntactically speaking). If you're using PyCharm (and you should) you'll
    notice a green check mark at the top right corner of the editor."""
    return args


def plot_problem(problem, ax=None, **kwargs):  # Here kwargs must be considered as a python dictionary
    """This function creates a plot representing the problem to solve. It plots the depot and the clients (and the
    clients' demand if plot_demand is True).
    Valid keyword argument:
    *plot_demand*: boolean, default True.
    *demand_size*: float, default 8. Size (in pixels) of the figures representing the clients' demands (if drawn)
    """
    if ax is None:  # in case no axis is provided
        fig = plt.figure()  # creates a window (without any axis). 1 figure = 1 window. 1 figure = 1 or more axes.
        ax = fig.add_subplot(111)  # creates one set of axes that takes all the space in the previously created window
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title("Number of clients = {}, total demand = {}".format(problem.number_of_clients, problem.total_demand))
    ax.grid(False)  # turns the grid off
    # You should complete the x_depot, y_depot, x_clients and y_clients variables. Each of these variables should be a
    # python list of floats.
    x_depot = [problem.depot.x]
    y_depot = [problem.depot.y]
    x_clients = []
    y_clients = []
    for client in problem.clients_list:
        x_clients.append(client.x)
        y_clients.append(client.y)
    # return x_depot, y_depot, x_clients, y_clients
    ax.plot(x_depot, y_depot, marker="s", color="red", label="Depot", linestyle="None", ms=7, zorder=2)
    ax.plot(x_clients, y_clients, marker="o", color="blue", label="Clients (demand)", linestyle="None", ms=3, zorder=1)
    if kwargs.get("plot_demand", True):  # returns the value of "plot_demand" if the key exists and True otherwise.
        for client in problem.clients_list:
            ax.text(client.x, client.y, str(client.demand), style="italic",
                    fontsize=kwargs.get("demand_size", 16), color="blue", ha="center", va="bottom", zorder=1)
    # plt.show()
    return ax


def plot_solution(solution, ax, color, **kwargs):
    """This function plots a solution. More precisely, it plots all the deliveries of the solution on the axis 'ax' with
    the color 'color'.
    :param solution: instance of class solution
    :param ax: instance of matplotlib axis
    :param color: valid matplotlib color. eg: 'b'=blue, 'r'=red, (1, 0, 0.5)=(r, g, b)=rose
    Valid keyword arguments:
    *dashed* : boolean, default True. If True the trajectory of the drone is plotted using a dashed pattern.
    *draw_wind*: boolean, default True. Arrows representing the wind are drawn if True
    *nx*: integer, default 10
    *ny*: integer, default 10. A grid of (nx-1)*(ny-1) evenly spaced arrows of wind are drawn on ax
    *rs*: float, default 0.1. The higher the number, the smaller the wind arrows get.
    """
    # setting up a dashed pattern. It represents a sequence of on/off ink (in points).
    dashes = [np.random.randint(12, 20+1), 2, np.random.randint(4, 6+1), 5]  # partially randomised
    # plotting the deliveries of the solution
    for i, delivery in enumerate(solution.deliveries_list):
        label = None
        if i == 0:
            label = solution.name
        # Here, you should generate the x_list and y_list variables
        x_list = [delivery.depot.x]
        y_list = [delivery.depot.y]
        for client in delivery.clients_list:
            x_list.append(client.x)
            y_list.append(client.y)
        x_list.append(delivery.depot.x)
        y_list.append(delivery.depot.y)
        line, = ax.plot(x_list, y_list, color=color, marker="", linestyle="-", label=label, zorder=0)
        if kwargs.get('dashed', True):
            line.set_dashes(dashes)
    ax.legend(loc=0, fontsize='x-small', numpoints=1)
    # Drawing arrows representing the wind
    if kwargs.get('draw_wind', True) and solution.parameters.wind.speed > 0:
        nx = kwargs.get('nx', 10)  # nx-1 is the number of arrows on the x axis
        ny = kwargs.get('ny', 10)  # ny-1 is the number of arrows on the y axis
        rs = kwargs.get('rs', 0.1)  # scaling value for the arrows (m.s-1)
        u = np.ones((nx, ny)) * solution.parameters.wind.x
        v = np.ones((nx, ny)) * solution.parameters.wind.y
        x = np.linspace(*ax.get_xbound(), nx + 2)[1:-1]
        y = np.linspace(*ax.get_ybound(), ny + 2)[1:-1]
        plt.quiver(x, y, u, v, angles='xy', units='dots', scale=rs,
                   width=2, headwidth=2, headlength=3.5, facecolor=color, edgecolor=color, zorder=-1, alpha=0.4)
    return ax


def get_random_color(color_list, **kwargs):
    """This function randomly generates a color that contrasts the most with the colors that are in color_list.
    This function actually generates n_color_candidates colors and selects the one that contrasts the most
    with the other colors.
    color_list is a python list of colors. A color is an array (or a list) of 3 floats (rgb values) between 0 and 1.
    Valid kwargs arguments:
    *n_color_candidates*: int, default 10. The final color is the best amongst the n_color_candidates candidate colors.
    """
    color = None
    n_color_candidates = kwargs.get('n_color_candidates', 10)
    color_candidates_matrix = np.random.rand(n_color_candidates, 3)  # creating matrix of candidate rgb values
    norm = 0.
    for i in range(n_color_candidates):
        candidate_color = color_candidates_matrix[i]
        candidate_norm = np.min([np.linalg.norm(existing_color-candidate_color) for existing_color in color_list])
        if candidate_norm > norm:
            norm = candidate_norm
            color = candidate_color
    color_list.append(color)
    return color


def plot_problem_solutions(problem, **kwargs):
    """This function plots a problem and its solutions using random colors. Returns the matplotlib axis where the plot
    has been made.
    Valid kwargs arguments:
    *ax*: instance of matplotlib axes, default None. If None is provided then a new figure + axis is created.
    Other keyword arguments are valid (see valid kwargs of plot_problem, get_random_color and plot_solution).
    """
    # You must use get_random_color to pick the random color.
    # Don't forget to pass kwargs to affected functions.
    colors_list = [np.array([1., 1., 1.])]  # [1., 1., 1.] = white (background color of the figure)
    ax = kwargs.get('ax')
    if ax is None:
        fig = plt.figure()
        ax = fig.add_subplot(111)
    plot_problem(problem, ax, **kwargs)
    # complete the function below
    # place_holder(problem, colors_list)
    # ax2 = plot_problem(problem)
    # if kwargs.get("solution", False):
    #     for solution in kwargs.get("solution"):
    #         ax2 = plot_solution(solution, ax2, get_random_color(colors_list))
    for solution in problem.solutions_list:
        ax = plot_solution(solution, ax, get_random_color(colors_list, **kwargs), **kwargs)
    return ax
