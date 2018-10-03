import numpy as np
import pyDroneDeliv.pre_processing as pre


def drone_power_consumption(drone, speed_rel_to_air, rho=1.3):
    """Returns the power consumption (expressed in watts) of the drone given its speed relative to the air.
    The power consumption is equal to the force of the air on the drone multiplied by the speed of the drone relative to
    the air.
    The force of the air on the drone is proportional to the square of its speed relative to the air.
    power = 1/2 * rho * A * Cd * speed^3
    :param drone: instance of class Drone
    :param speed_rel_to_air: float value (m.s-1)
    :param rho: air density (kg/m3)
    :return float value representing the power consumption
    """
    # pre.place_holder(drone, speed_rel_to_air, rho)
    power = 1/2 * rho * drone.acd * speed_rel_to_air ** 3
    return power


def cost_a(point_a, point_b, drone, wind):
    """
    Returns the cost (expressed in joules) from point a to point b for a given drone and a given wind.
    In this formula, the drone is considered to be moving at constant speed relative to the GROUND
    (drone.speed = speed of the drone relative to the ground).
    The drone flies in a straight line from a to b.
    :param point_a: instance of class Point
    :param point_b: instance of class Point
    :param drone: instance of class Drone
    :param wind: instance of class Wind
    :return: float. Cost from a to b (J).
    """
    # pre.place_holder(point_a, point_b, drone, wind)
    assert drone.speed > 0.  # verifies that the drone can actually move. It raises an AssertError otherwise.
    vecteur_deplacement = np.array((point_b.x-point_a.x, point_b.y-point_a.y))
    distance = np.linalg.norm(vecteur_deplacement)
    if distance > 0:
        vecteur_vitesse_drone_sol = drone.speed * vecteur_deplacement / distance
        vecteur_vitesse_drone_air = vecteur_vitesse_drone_sol - wind.vector
        norm_vitesse_relative = np.linalg.norm(vecteur_vitesse_drone_air)
        cost = drone_power_consumption(drone, norm_vitesse_relative, rho=1.3) * distance / drone.speed
        return cost
    else:
        return 0


def cost_b(point_a, point_b, drone, wind, safety_factor=2):
    """
    Returns the cost (expressed in joules) from point a to point b for a given drone and a given wind.
    In this formula, the drone is considered to be moving at constant speed relative to the AIR
    (drone.speed = speed of the drone relative to the air).
    The speed of the drone must be strictly greater than that of the wind with a certain margin or an AssertionError
    is raised.
    The drone flies in a straight line from a to b.
    :param point_a: instance of class Point
    :param point_b: instance of class Point
    :param drone: instance of class Drone
    :param wind: instance of class Wind
    :param safety_factor: float. must be strictly greater than 1. The drone must go safety_factor times faster than the
    wind.
    :return: float. Cost from a to b (J).
    """
    # You can safely skip this function if you want. It will not prevent you to pass the automatic evaluation of the
    # rest of the DEV tasks.
    # pre.place_holder(point_a, point_b, drone, wind)
    assert safety_factor > 1  # verifies that the safety_factor is greater than 1.
    assert drone.speed > safety_factor*wind.speed
    vecteur_deplacement = np.array((point_b.x - point_a.x, point_b.y - point_a.y))
    distance = np.linalg.norm(vecteur_deplacement)
    if distance > 0:
        vecteur_deplacement_norme = vecteur_deplacement / distance
        e = (wind.x * vecteur_deplacement_norme[0] + wind.y * vecteur_deplacement_norme[1])
        f = wind.speed**2 - drone.speed**2
        delta = 4*e**2 - 4*f
        v3 = e + (0.5 * np.sqrt(delta))
        # vecteur_vitesse_drone_air = drone.speed * vecteur_deplacement / distance
        # vecteur_vitesse_drone_sol = vecteur_vitesse_drone_air + wind.vector
        # norm_vitesse_drone_sol = np.linalg.norm(vecteur_vitesse_drone_sol)
        # v3 = + drone.speed - np.vdot(wind.vector, vecteur_deplacement/distance)
        return drone_power_consumption(drone, drone.speed, rho=1.3) * distance / v3
    else:
        return 0


def check_route_compatibility(route_a, route_b):
    """Checks if two routes don't have inherent incompatibilities. Returns True if no incompatibility and False
    otherwise."""
    # Right now we'll consider that two routes are compatible is they have the same depot. It might change in the
    # future with new developments.
    # pre.place_holder(route_a, route_b)
    if route_a.depot == route_b.depot:
        return True
    else:
        return False


def check_delivery_compatibility(delivery_a, delivery_b):
    """Checks if two deliveries don't have inherent incompatibilities. Returns True if no incompatibility and False
    otherwise."""
    # We'll consider that two deliveries are compatible if their routes are compatibles and it they have the same drone
    # and the same wind.
    # pre.place_holder(delivery_a, delivery_b)
    if delivery_a.drone == delivery_b.drone and delivery_a.wind == delivery_b.wind and \
            check_route_compatibility(delivery_a.route, delivery_b.route):
        return True
    else:
        return False


def merge_routes(route_a, route_b, must_have_common_client=False):
    """Merges route_a and route_b into a new route where the list of clients is:
    -first, the list of clients of route_a
    -then, the list of clients of route_b
    For the new route to be legal:
    - route_a and route_b must be compatible
    - the list of clients of the new route must be legal
    One subtlety:
    - the last client of route_a and the first client of route_b can be identical but it is not mandatory unless
    must_have_common_client is true. Should route_a and route_b have this common client, the list of clients of the
    merged route would, of course, not have this common client duplicated.
    Returns the new route if legal. Returns None otherwise."""
    # pre.place_holder(route_a, route_b, must_have_common_client)
    if not check_route_compatibility(route_a, route_b):
        return None
    if check_route_compatibility(route_a, route_b):
        list_des_clients = []
        if must_have_common_client:
            if len(route_a.clients_list) == 0 or len(route_b.clients_list) == 0:
                return None
            if not route_a.clients_list[-1] is route_b.clients_list[0]:
                return None
            for i in range(0, len(route_a.clients_list)):
                list_des_clients.append(route_a.clients_list[i])
            for k in range(1, len(route_b.clients_list)):
                list_des_clients.append(route_b.clients_list[k])
        if not must_have_common_client:
            if len(route_a.clients_list) != 0 and len(route_b.clients_list) != 0:
                if route_a.clients_list[-1] is route_b.clients_list[0]:
                    for i in range(0, len(route_a.clients_list)):
                        list_des_clients.append(route_a.clients_list[i])
                    for k in range(1, len(route_b.clients_list)):
                        list_des_clients.append(route_b.clients_list[k])
            if not list_des_clients:
                for client in route_a.clients_list:
                    list_des_clients.append(client)
                for client2 in route_b.clients_list:
                    list_des_clients.append(client2)
        route_c = pre.Route(list_des_clients, route_a.depot)
        if route_c.is_legal:
            return route_c
        if not route_c.is_legal:
            return None


def merge_deliveries(delivery_a, delivery_b, must_have_common_client=False):
    """Merges delivery_a and delivery_b into a new delivery where the route is generated using merge_routes rules.
    delivery_a and delivery_b must be compatible.
    Returns the new delivery if legal. Returns None otherwise."""
    # pre.place_holder(delivery_a, delivery_b, must_have_common_client)
    if not check_delivery_compatibility(delivery_a, delivery_b):
        return None
    if check_delivery_compatibility(delivery_a, delivery_b):
        new_route = merge_routes(delivery_a.route, delivery_b.route, must_have_common_client)
        if new_route:
            new_delivery = pre.Delivery(new_route, delivery_a.parameters)
            if new_delivery.is_legal:
                return new_delivery
        else:
            return None


def cost_matrix(problem, parameters):
    """This function returns the cost matrix of a problem for a given parameter set.
    :param problem: Instance of class Problem
    :param parameters: Instance of class DeliveryParameters
    :return: 2-dimensional numpy array representing the cost matrix"""
    # pre.place_holder(problem, parameters)
    mat_dim = problem.number_of_clients + 1
    c_matrix = np.zeros((mat_dim, mat_dim))  # creates a square matrix (2-dimensional numpy array) filled with zeros
    if parameters.cost_fct:
        for i in range(0, len(problem.clients_list)):
            c_matrix[0][i + 1] = parameters.cost_fct(problem.depot, problem.clients_list[i],
                                                     parameters.drone, parameters.wind)
            c_matrix[i + 1][0] = parameters.cost_fct(problem.clients_list[i], problem.depot,
                                                     parameters.drone, parameters.wind)
            for k in range(0, len(problem.clients_list)):
                c_matrix[i+1][k+1] = parameters.cost_fct(problem.clients_list[i], problem.clients_list[k],
                                                         parameters.drone, parameters.wind)
    else:
        for i in range(0, len(problem.clients_list)):
            for k in range(i+1, len(problem.clients_list)+1):
                c_matrix[i][k] = c_matrix[k][i] = None
    return c_matrix


def savings_matrix(problem, parameters):
    """This function returns the savings matrix of a problem for a given parameter set.
    :param problem. Instance of class Problem
    :param parameters. Instance of class DeliveryParameters
    :return 2-dimensional numpy array representing the savings matrix"""
    # pre.place_holder(problem, parameters)
    mat_dim = problem.number_of_clients
    c_matrix = cost_matrix(problem, parameters)
    s_matrix = np.zeros((mat_dim, mat_dim))  # creates a square matrix (2-dimensional numpy array) filled with zeros
    if parameters.cost_fct:
        for i in range(0, len(problem.clients_list)):
            for k in range(0, len(problem.clients_list)):
                if k != i:
                    a = parameters.cost_fct(problem.clients_list[i], problem.depot, parameters.drone, parameters.wind)
                    b = parameters.cost_fct(problem.depot, problem.clients_list[k], parameters.drone, parameters.wind)
                    s_matrix[i][k] = a + b - c_matrix[i+1][k+1]
    return s_matrix


def clarke_and_wright_init(problem, parameters):
    """This function initializes the Clarke and Wright algorithm.
    This function calculates the savings matrix and returns a tuple (sorted_savings, client_pairs) where:
    sorted_savings = the sorted savings in a one-dimensional numpy array (sorted in descending order)
    client_pairs = list of tuples in the form (client_i, client_j) where client_i and client_j are clients of the
    problem. client_i and client_j of the k-th tuple represent the clients associated with the k-th value of
    sorted_savings."""
    # look for the numpy methods 'flatten' and 'argsort'.
    # pre.place_holder(problem, parameters)
    if parameters.cost_fct:
        the_list = savings_matrix(problem, parameters).flatten()
        indice_list = np.argsort(the_list)
        sorted_savings = []
        clients_pairs = []
        for j in range(0, len(indice_list)):
            k = indice_list[j]
            sorted_savings.append(the_list[k])
            a = k // len(problem.clients_list)
            b = k % len(problem.clients_list)
            # problem.clients_list[a].identifier = "client_{}".format(a)
            # problem.clients_list[b].identifier = "client_{}".format(b)
            clients_pairs.append((problem.clients_list[a], problem.clients_list[b]))
            # clients_pairs.append(({client_{}}.format(problem.clients_list[a]),{}.format(problem.clients_list[b]))
        sorted_savings.reverse()
        clients_pairs.reverse()
        np_sorted_savings = np.array(sorted_savings)
        return np_sorted_savings, clients_pairs
    else:
        return [], []


def add_single_client_deliveries(deliveries_list, problem, parameters):
    """This function modifies the deliveries_list argument by adding deliveries containing a single client.
    Only clients that are present in the problem but not present in any delivery are added. It first checks that the
    client can legally be delivered."""
    # pre.place_holder(deliveries_list, problem, parameters)
    list_of_clients = []
    for delivery in deliveries_list:
        for j in range(len(delivery.clients_list)):
            list_of_clients.append(delivery.clients_list[j])
    absent_clients = []
    for client in problem.clients_list:
        client_number_of_apparence = 0
        for client_already_present in list_of_clients:
            if client == client_already_present:
                client_number_of_apparence += 1
        if client_number_of_apparence == 0:
            absent_clients.append(client)
    # return absent_clients
    for absent_one in absent_clients:
        route_absent_one = pre.Route([absent_one], problem.depot)
        delivery_absent_one = pre.Delivery(route_absent_one, parameters)
        if delivery_absent_one.is_legal:
            deliveries_list.append(delivery_absent_one)
    return deliveries_list


def sequential_merge_if_possible(delivery_a, delivery_b):
    """This function tries to merge two deliveries if possible in the sequential version of Clarke and Wright
    (ie the two deliveries MUST have a common client at their borders except if at least one of the deliveries is empty)
    It returns the merged delivery if possible. Returns None if the two deliveries can't be legally merged."""
    if len(delivery_a.clients_list) == 0 or len(delivery_b.clients_list) == 0:
        return merge_deliveries(delivery_a, delivery_b, False)
    new_delivery = merge_deliveries(delivery_a, delivery_b, True)
    if new_delivery is not None:
        return new_delivery
    return merge_deliveries(delivery_b, delivery_a, True)


def search_deliveries_for_client(client, deliveries_list, include_first=True, include_interior=True, include_last=True):
    """This function searches for a specified client in a list of deliveries. The search is performed at the borders
    and/or in the interior of the deliveries according to the values of the parameters.
    The function returns the first delivery where the client has been found.
    It returns None if the client has not been found in any of the deliveries."""
    for delivery in deliveries_list:
        nb_clients = len(delivery.clients_list)
        if include_interior and nb_clients >= 3:
            if client in delivery.clients_list[1:-1]:
                return delivery
        if include_first and nb_clients >= 1:
            if client == delivery.clients_list[0]:
                return delivery
        if include_last and nb_clients >= 1:
            if client == delivery.clients_list[-1]:
                return delivery
    return None


def sequential_build_deliveries(problem, parameters, sorted_savings, client_pairs):
    """This function returns a list of instances of class Delivery calculated with the use of the sequential Clarke
    and Wright algorithm. Single client deliveries are added at the end if necessary."""
    # pre.place_holder(problem, parameters, sorted_savings, client_pairs)
    deliveries_list = []
    j = 0
    # clients_restants = problem.clients_list
    client_pairs_modifie = client_pairs
    while j < len(client_pairs)+1:
        i = -1
        a = 0
        k = 0
        while k < len(client_pairs_modifie):
            pair = client_pairs_modifie[k]
            if sorted_savings[k] >= 0:
                if i < 0:   # indicateur pour voir si on vient de commencer le nouvel itinéraire ou pas
                    route_a = pre.Route([], problem.depot)
                    delivery_a = pre.Delivery(route_a, parameters)
                else:
                    delivery_a = deliveries_list[-1]
                route_b = pre.Route([pair[0], pair[1]], problem.depot)
                delivery_b = pre.Delivery(route_b, parameters)
                delivery_c = sequential_merge_if_possible(delivery_a, delivery_b)
                # if k == 3 and j == 1:
                #     print (4, delivery_a.clients_list, delivery_b.clients_list, delivery_c)
                if delivery_c:
                    if i < 0:
                        deliveries_list.append(delivery_c)
                        i += 1
                    else:
                        deliveries_list[-1] = delivery_c
                        k = 0
                    # a = 0
                if not delivery_c:
                    a += 1
            k += 1
        if a == len(client_pairs_modifie):
            add_single_client_deliveries(deliveries_list, problem, parameters)
            # for delivery in deliveries_list:
            #     delivery.print()
            return deliveries_list
        client_pairs_modifie = []
        for k, pair in enumerate(client_pairs):
            if (search_deliveries_for_client(pair[0], deliveries_list) is None) and \
                    (search_deliveries_for_client(pair[1], deliveries_list) is None):
                client_pairs_modifie.append(pair)
        # if j == 0:
        #     print(client_pairs_modifie)
        #     # c'est la qu'on voit l'ordre des couples adapté après la 1ère boucle
        # clients_restants = []
        # for client in problem.clients_list:
        #     if search_deliveries_for_client(client, deliveries_list) is None:
        #         clients_restants.append(client)
        j += 1


def parallel_build_deliveries(problem, parameters, sorted_savings, client_pairs):
    """This function returns a list of instances of class Delivery calculated with the use of the parallel Clarke
    and Wright algorithm. Single client deliveries are added at the end if necessary."""
    # pre.place_holder(problem, parameters, sorted_savings, client_pairs)
    deliveries_list = []
    for k, pair in enumerate(client_pairs):
        # print(k)
        if sorted_savings[k] >= 0:
            route_b = pre.Route([pair[0], pair[1]], problem.depot)
            delivery_b = pre.Delivery(route_b, parameters)
            if delivery_b.is_legal:
                if len(deliveries_list) == 0:
                    deliveries_list.append(delivery_b)
                else:
                    route_b = pre.Route([pair[0], pair[1]], problem.depot)
                    delivery_b = pre.Delivery(route_b, parameters)
                    if search_deliveries_for_client(pair[0], deliveries_list, False, False, True) and \
                            search_deliveries_for_client(pair[1], deliveries_list, True, False, False):
                        delivery_c = search_deliveries_for_client(pair[0], deliveries_list, False, False, True)
                        delivery_d = search_deliveries_for_client(pair[1], deliveries_list, True, False, False)
                        delivery_e = sequential_merge_if_possible(delivery_b, delivery_c)
                        # delivery_f = sequential_merge_if_possible(delivery_b, delivery_d)
                        if delivery_e and delivery_d:
                            delivery_z = sequential_merge_if_possible(delivery_e, delivery_d)
                            if delivery_z:
                                deliveries_list.append(delivery_z)
                                deliveries_list.remove(delivery_c)
                                deliveries_list.remove(delivery_d)
                    elif not search_deliveries_for_client(pair[0], deliveries_list, False, False, True) and \
                            search_deliveries_for_client(pair[1], deliveries_list, True, False, False):
                        delivery_d = search_deliveries_for_client(pair[1], deliveries_list, True, False, False)
                        delivery_z = sequential_merge_if_possible(delivery_b, delivery_d)
                        if delivery_z and not search_deliveries_for_client(pair[0], deliveries_list):
                            deliveries_list.append(delivery_z)
                            deliveries_list.remove(delivery_d)
                    elif search_deliveries_for_client(pair[0], deliveries_list, False, False, True) and not \
                            search_deliveries_for_client(pair[1], deliveries_list, True, False, False):
                        delivery_c = search_deliveries_for_client(pair[0], deliveries_list, False, False, True)
                        delivery_z = sequential_merge_if_possible(delivery_c, delivery_b)
                        if delivery_z and not search_deliveries_for_client(pair[1], deliveries_list):
                            deliveries_list.append(delivery_z)
                            deliveries_list.remove(delivery_c)
                    elif not search_deliveries_for_client(pair[0], deliveries_list, False, False, True) and not \
                            search_deliveries_for_client(pair[1], deliveries_list, True, False, False):
                        if not search_deliveries_for_client(pair[0], deliveries_list) and not \
                                search_deliveries_for_client(pair[1], deliveries_list) and delivery_b:
                            deliveries_list.append(delivery_b)
    add_single_client_deliveries(deliveries_list, problem, parameters)
    # for delivery in deliveries_list:
    #     delivery.print()
    return deliveries_list


def build_deliveries(problem, parameters, version, sorted_savings, client_pairs):
    """Returns a list of deliveries resulting from the use of the Clarke and Wright algorithm.
    :param problem: problem to solve
    :param parameters: parameters of the deliveries
    :param version: version of the Clarke and Wright algorithm. Must be "Sequential" or "Parallel".
    :param sorted_savings: list of the savings sorted in descending order. Result of clarke_and_wright_init.
    :param client_pairs: list of pairs of clients relative to sorted_savings. Result of clarke_and_wright_init.
    :return list: list of instances of class Delivery.
    """
    if version == "sequential":
        return sequential_build_deliveries(problem, parameters, sorted_savings, client_pairs)
    if version == "parallel":
        return parallel_build_deliveries(problem, parameters, sorted_savings, client_pairs)
    return []


def clarke_and_wright(problem, parameters, version="sequential", name=None, verbose=True):
    """Solves a problem using the clarke and Wright algorithm. Creates a solution and appends it to the end of the
    solutions list of the problem."""
    if version != "sequential" and version != "parallel":
        print("Unexpected version : {}".format(version))
        print("Please use 'sequential' or 'parallel'")
        return
    if name is None:
        name = version + " Clarke and Wright. Drone capacity = {}".format(parameters.drone.capacity)

    if verbose:
        print("Initialising Clarke & Wright {} version...".format(version), end=' ', flush=True)
    init = clarke_and_wright_init(problem, parameters)
    if verbose:
        print("done !")

        print("Building deliveries...".format(version), end=' ', flush=True)
    deliveries_list = build_deliveries(problem, parameters, version, *init)
    problem.solutions_list.append(pre.Solution(name, deliveries_list, parameters))
    if verbose:
        print("done !")
        problem.solutions_list[-1].print(False)

