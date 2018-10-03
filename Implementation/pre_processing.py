"""Implements all the necessary classes for this project."""
import csv  # module from the standard library used for reading and writing files in CSV (Coma Separated Values) format.
import numpy as np  # numpy is a very popular and powerful module. Use it every time you need to deal with numbers.


def place_holder(*args):
    """This function does nothing. It is just a place holder that is used to fill the 'holes' in the code. Its only
    purpose is to make the code look nice (syntactically speaking)."""
    # If you're using PyCharm (and you should) you'll notice a green check mark at the top right corner of the editor.
    # While you're coding, you need to make sure that you don't lose this green check mark.
    return args


class Drone:
    """This class represents a drone. It has 3 attributes: 'capacity', 'speed' and 'acd' and one method: 'copy'."""

    def __init__(self, capacity=100, speed=10., acd=0.01):  # Overwriting the __init__ method is a very common practice.
        # Default values are:
        # capacity, 100 units (the physical unit doesn't matter in this study)
        # speed, 10 m.s-1
        # acd, 0.01 m2
        self.capacity = capacity  # int. Capacity of the drone.
        self.speed = speed  # float. Speed (m.s-1).
        self.acd = acd  # float. = A*Cd. A=cross sectional area (m2), Cd=drag coefficient.

    def __repr__(self):  # This is an other special method. Overwriting __repr__ is also optional but recommended.
        """Returns a string that is a formal representation of an instance of this class."""
        # id(self) returns a long integer representing its address in the memory. eg: 73366096
        # hex(some_integer) returns some_integer in its hexadecimal form. eg: 0x45f7a50
        return "<Drone at {}. capacity={}, speed={}, acd={}>".format(hex(id(self)), self.capacity, self.speed, self.acd)

    def copy(self, other_drone):  # This is called a 'method' of the class.
        """Modifies the values of the attributes of this instance to match the ones of other_drone."""
        # It is perfectly legal to call an other method (even a special one) from inside a method.
        self.__init__(other_drone.capacity, other_drone.speed, other_drone.acd)


class Wind:
    """This class models the wind. It has 2 attributes 'x' and 'y'. It has 2 properties 'vector' and 'speed'."""

    def __init__(self, x=0., y=0.):  # It is mandatory to define a __init__ method.
        self.x = x  # float. speed (m.s-1) on the x axis (x>0 means the wind is blowing from west to east)
        self.y = y  # float. speed (m.s-1) on the y axis (y>0 means the wind is blowing from south to north)

    def __repr__(self):
        return "<Wind at {}. (x, y) = ({}, {}) ; speed = {} m.s-1>".format(hex(id(self)), self.x, self.y, self.speed)

    @property  # <- transforms the method into an attribute
    def vector(self):  # You can thus call some_wind.vector without the parenthesis as if it was just another attribute.
        """Returns the x and y attributes as a 1-dimensional numpy array."""
        return np.array((self.x, self.y))

    @vector.setter  # <- This is the setter function of the 'vector' property.
    def vector(self, xy):  # It allows you to define what happens when you type 'some_wind.vector = something'
        """Allows to set both self.x and self.y in one shot."""
        self.x = xy[0]
        # to be continued ...
        self.y = xy[1]

    @property
    def speed(self):
        """Returns the speed of the wind (m.s-1)."""
        return np.linalg.norm(self.vector)

    @speed.setter
    def speed(self, target_speed):
        if self.speed > 0:
            self.vector = (target_speed / self.speed) * self.vector
        else:
            self.x = target_speed


class Point:
    """This class represents a point on the map. 'x' and 'y' are the coordinates of the point."""

    def __init__(self, identifier="", x=0., y=0.):
        self.identifier = identifier  # string
        self.x = x  # float. x coordinate (m)
        self.y = y  # float. y coordinate (m)

    def __repr__(self):
        """Formal representation of an instance of this class."""
        return "<Point at {}. id = {}, (x,y) = ({}, {})>".format(hex(id(self)), self.identifier, self.x, self.y)

    def __str__(self):
        """Informal representation of an instance of this class."""
        return self.identifier

    def copy(self, other_point):
        self.__init__(other_point.identifier, other_point.x, other_point.y)


class Client(Point):  # <- (Point) means that this class inherits from the class Point.
    """This class inherits from class Point. It adds an attribute : 'demand' which represents the client's demand."""

    def __init__(self, identifier="Client", x=0., y=0., demand=0):
        Point.__init__(self, identifier, x, y)  # <- defines 'x' and 'y' so there is no need to define them again.
        self.demand = demand  # int. New attribute that instances of class Client will have but instances of Point won't

    def __repr__(self):
        return "<Client at {}. id = {}, (x,y) = ({}, {}), demand = {}>".format(  # you can split a line in two...
            hex(id(self)), self.identifier, self.x, self.y, self.demand)  # ...just like this

    def copy(self, other_client):
        self.__init__(other_client.identifier, other_client.x, other_client.y, other_client.demand)


class Depot(Point):
    """This class inherits from Point. It actually doesn't change anything except the default value of the identifier"""

    def __init__(self, identifier="Depot", x=0., y=0.):
        Point.__init__(self, identifier, x, y)

    def __repr__(self):
        return "<Depot at {}. id = {}, (x,y) = ({}, {})>".format(hex(id(self)), self.identifier, self.x, self.y)


class Route:
    def __init__(self, clients_list, depot):
        self.clients_list = clients_list  # python list of instances of class Client
        self.depot = depot  # instance of class Depot. A route always starts and ends at a depot.

    def __repr__(self):  # formal representation of an instance of this class
        return "<Route at {}. [".format(hex(id(self))) + ", ".join([repr(cl) for cl in self.clients_list]) + \
               "]. {}.>".format(repr(self.depot))

    def __str__(self):  # informal representation. It replaces __repr__ when calling the print function.
        return "Route at {}. [".format(hex(id(self))) + ", ".join([str(cl) for cl in self.clients_list]) + \
               "]. {}.".format(str(self.depot))

    @property
    def total_demand(self):
        """This method returns the total demand of all the clients of the route."""
        total = 0
        for i in range(0, len(self.clients_list)):
            total += self.clients_list[i].demand
        return total

    @property
    def is_legal(self):
        """This method returns True if the route does not break any rule of the delivery problem. It returns False
        otherwise."""
        # Assume that self.depot is actually of type Depot.
        # Assume that all instances in self.clients_list are actually of type Client.
        for i in range(0, len(self.clients_list) - 1):
            for j in range(i + 1, len(self.clients_list)):
                if self.clients_list[i] == self.clients_list[j]:
                    return False
        return True

    def check_same_depot(self, other_route):
        """This method returns True if this route has the same depot as other_route and False otherwise."""
        # It must be the same depot instance and not just two identical depots.
        if self.depot == other_route.depot:
            return True
        else:
            return False


class DeliveryParameters:
    def __init__(self, drone, wind, cost_fct=None):
        """
        :param drone: instance of class Drone
        :param wind: instance of class Wind
        :param cost_fct: cost function that computes the cost to go from one point to another as a function of the
        drone and the wind"""
        # see cost_a and cost_b from processing.py
        self.drone = drone
        self.wind = wind
        self.cost_fct = None
        if cost_fct is not None:
            self.cost_fct = cost_fct


class Delivery:
    def __init__(self, route, parameters):
        self.route = route  # instance of class Route
        self.parameters = parameters  # instance of class DeliveryParameters.

    @property
    def clients_list(self):
        return self.route.clients_list

    @clients_list.setter
    def clients_list(self, new_list):
        self.route.clients_list = new_list

    @property
    def total_demand(self):
        return self.route.total_demand

    @property
    def depot(self):
        return self.route.depot

    @property
    def drone(self):
        return self.parameters.drone

    @property
    def wind(self):
        return self.parameters.wind

    @property
    def is_legal(self):
        """This method returns True if the delivery does not break any rule of the delivery problem. It returns False
        otherwise."""
        if self.route.is_legal and self.route.total_demand <= self.drone.capacity:
            return True
        else:
            return False

    def cost(self):
        """Returns the cost of the delivery according to its cost function. Returns None if its cost function is None.
        """
        # It is not necessary to program cost_a nor cost_b (in processing.py) to program this method and pass the
        # automatic evaluation.
        if self.parameters.cost_fct is None:
            return None
        else:
            cost = 0
            a = len(self.clients_list)
            if a == 0:
                return 0
            else:
                cost += self.parameters.cost_fct(self.depot, self.clients_list[0], self.drone, self.wind)
                cost += self.parameters.cost_fct(self.clients_list[-1], self.depot, self.drone, self.wind)
                if a == 1:
                    return cost
                if a > 1:
                    for i in range(0, len(self.clients_list) - 1):
                        cost += self.parameters.cost_fct(self.clients_list[i], self.clients_list[i + 1], self.drone,
                                                         self.wind)
                    return cost


    def cost_and_savings(self):
        """Returns a tuple (cost, savings) corresponding to the cost and the savings of the delivery according to its
        cost function. Returns the tuple (None, None) if the cost function of the delivery is None."""
        if self.parameters.cost_fct is None:
            return None, None
        else:
            normalcost = 0
            a = len(self.clients_list)
            if a <= 1:
                return self.cost(), 0
            else:
                for i in range(0, len(self.clients_list)):
                    normalcost += self.parameters.cost_fct(self.clients_list[i], self.depot, self.drone, self.wind)
                    normalcost += self.parameters.cost_fct(self.depot, self.clients_list[i], self.drone, self.wind)
                savings = normalcost - self.cost()
                return self.cost(), savings

    # methods
    def print(self):
        print("Delivery at {}:".format(hex(id(self))))
        print("   ", self.route)
        print("    total demand = {}, drone capacity = {}, wind = {}, cost = {:.5e}, savings = {:.5e}"
              .format(self.total_demand, self.drone.capacity, self.wind.vector, *self.cost_and_savings()))


class Solution:
    """A solution is basically a combination of deliveries (in the form of a list of deliveries)."""

    def __init__(self, name="Unnamed Solution", deliveries_list=None, parameters=None):
        self.name = name  # string. Useful for identification and post-processing
        self.deliveries_list = list()
        if deliveries_list is not None:
            self.deliveries_list = deliveries_list  # python list of instances of class Delivery.
        self.parameters = parameters  # instance of class DeliveryParameters

    @property
    def is_legal(self):  # useful for debugging purposes
        """This method returns True if the solution does not break any rule of the delivery problem. It returns False
        otherwise."""
        a = len(self.deliveries_list)
        for i in range(0, a):
            if self.deliveries_list[i].is_legal is False:
                return False
        ctot = []
        for i in range(0, a):
            for j in self.deliveries_list[i].clients_list:
                ctot.append(j)
        lctot = len(ctot)
        for k in range(0, lctot - 1):
            for l in range(k + 1, lctot):
                if ctot[k] == ctot[l]:
                    return False
        return True

    def cost_and_savings(self):
        """Returns the total cost and total savings of the solution. Deliveries without cost function are ignored.
        Returns (0, 0) if there is no delivery or if all the deliveries are without a cost function."""
        if not self.deliveries_list:
            return 0, 0
        else:
            cost = 0
            savings = 0
            length = len(self.deliveries_list)
            for i in range(0, length):
                if self.deliveries_list[i].parameters.cost_fct:
                    interest = self.deliveries_list[i].cost_and_savings()
                    cost += interest[0]
                    savings += interest[1]
            return cost, savings

    def print(self, detailed=True):
        print("Solution name : {}. Number of deliveries = {}. Total cost = {:.5e}, total savings = {:.5e}".format(
            self.name, len(self.deliveries_list), *self.cost_and_savings()))
        if detailed:
            for delivery in self.deliveries_list:
                delivery.print()


class Problem:
    """A problem is a list of clients to deliver from a given depot.
    This class can also store a list of solutions."""

    def __init__(self, depot=None, clients_list=None):
        self.depot = depot
        self.clients_list = list()
        if clients_list is not None:
            self.clients_list = clients_list  # python list of instances of class Client
        self._number_of_generated_clients = 0  # int. Useful for random problem generation.
        self.solutions_list = list()  # python list of instances of class Solution.

    @property
    def number_of_generated_clients(self):
        return self._number_of_generated_clients

    @property
    def number_of_clients(self):
        return len(self.clients_list)

    @property
    def total_demand(self):
        total = 0
        for client in self.clients_list:
            if client:
                total += client.demand
        return total

    def print_clients(self):
        for client in self.clients_list:
            print(repr(client))

    def print_depot(self):
        print(repr(self.depot))

    def print_solutions(self, detailed=False):
        for solution in self.solutions_list:
            solution.print(detailed)
            print("\n\n")

    def remove_solution_index(self, index):
        del self.solutions_list[index]

    def remove_solution_named(self, name):
        for i, solution in enumerate(self.solutions_list):
            if solution["Name"] == name:
                del self.solutions_list[i]
                break

    def clear_solutions(self):
        self.solutions_list.clear()

    def generate_random_clients(self, amount=1, x=(-10000, 10000), y=(-10000, 10000), demand=(1, 100)):
        """This method adds random clients to the end of the current list of clients.
        Every time a new client is generated, _number_of_generated_clients is increased by 1.
        When a client is generated, its identifier is "random client X" with X=_number_of_generated_clients.
        The x and y coordinates of the client are randomly chosen between the limits given by the 'x' and 'y' parameters
        respectively (m).
        The client's demand is also chosen randomly according to the 'demand' parameter (borders are inclusive).
        The demand is an integer, not a float."""
        # see np.random.rand and np.random.randint
        # Warning: the identifier must be EXACTLY int the form "random client X". eg: "random client 18".
        # Warning: the automatic evaluation is case sensitive ! "random client" is not the same as "Random Client".
        # place_holder(self, amount, x, y, demand)
        for i in range(1, amount + 1):
            self._number_of_generated_clients += 1
            client_i = Client("random client {}".format(self._number_of_generated_clients),
                              ((x[1] - x[0]) * (np.random.rand(1)) + x[0])[0],
                              ((y[1] - y[0]) * (np.random.rand(1)) + y[0])[0],
                              (np.random.randint(demand[0], demand[1] + 1, 1))[0])
            self.clients_list.append(client_i)

    def export_csv(self, file_name, cell_separator=";"):
        """This method exports the problem to a file in csv format.
        file_name is a string"""
        with open(file_name, "w", newline='') as f:
            writer = csv.writer(f, delimiter=cell_separator)
            writer.writerow(["Delivery optimization problem"])
            writer.writerow(["type", "identifier", "x", "y", "demand"])
            writer.writerow(["depot", self.depot.identifier, self.depot.x, self.depot.y])
            for client in self.clients_list:
                writer.writerow(["client", client.identifier, client.x, client.y, client.demand])

    def import_csv(self, file_name, cell_separator=";"):
        """This method reads a problem from a file in csv format.
        file_name is a string"""
        with open(file_name, newline='') as f:
            reader = csv.reader(f, delimiter=cell_separator)
            new_clients_list = list()
            for i, row in enumerate(reader):
                if i == 0 and row[0] != "Delivery optimization problem":
                    raise (FileExistsError, "Incorrect file type.")
                if i >= 2:
                    if row[0] == "depot":
                        self.depot = Depot(row[1], float(row[2]), float(row[3]))
                    if row[0] == "client":
                        new_clients_list.append(Client(row[1], float(row[2]), float(row[3]), int(row[4])))
                        if "random client" in row[1]:
                            self._number_of_generated_clients += 1
            self.clients_list = new_clients_list
