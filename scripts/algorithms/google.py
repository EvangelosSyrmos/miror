#!/usr/bin/env python
from  __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import sys
import time
import random
import matplotlib.pyplot as plt


class Selector(object):
    """Select which algorithms to run in order to get the best distance"""

    def __init__(self, distance_matrix, algorithms):
        self.distance_matrix = distance_matrix
        self.algorithms = algorithms
        self.alg_list = []

        for algo in self.algorithms:
            self.alg_list.append(Google(self.distance_matrix, algo))
        
        #region Best timer, distance
        self.best_timer = self.alg_list[0]
        self.best_distance = self.alg_list[0]

        for algo in self.alg_list:
            if algo._distance < self.best_distance._distance:
                self.best_distance = algo
            if algo.calc_time < self.best_timer.calc_time:
                self.best_timer = algo

        # print("Best distance --> {}".format(self.best_distance))
        # print("Best time --> {}".format(self.best_timer))
        #endregion


        
    def get_best_path(self):
        choices = [self.best_distance, self.best_timer]

        if self.best_distance is self.best_timer:
            return self.best_distance.routes
        else:
            return random.choice(choices)
                


class Google(object):
    
    def __init__(self, distance_matrix, name):

        self.routes = []

        temp = []
        temp = [[int(col) for col in row]for row in distance_matrix] # Typecasting distance matrix --> Int

        #region Create Data model
        self.data = {}
        self.data['distance_matrix'] = temp
        # print(self.data['distance_matrix'])
        self.data['num_vehicles'] = 1
        self.data['depot'] = 0
        #endregion

        self.start = time.time()

        #region API
        #Create routing index manager
        self.manager = pywrapcp.RoutingIndexManager(len(self.data['distance_matrix']), self.data['num_vehicles'], self.data['depot'])

        #Create routing Model
        self.routing = pywrapcp.RoutingModel(self.manager)

        self.transit_callback_index = self.routing.RegisterTransitCallback(self.distance_callback)

        #Define cost of each arc.
        self.routing.SetArcCostEvaluatorOfAllVehicles(self.transit_callback_index)

        # Setting first solution heuristic
        self.search_parameters = pywrapcp.DefaultRoutingSearchParameters()

        
        self._distance = 0
        self.name = name.upper()
        print("###################################")

        #region First solutions
        if self.name.upper() == "AUTOMATIC":
            print("Running: {}".format(self.name.title()))
            self.search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC)
        elif self.name.upper() == "SAVINGS":
            self.search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.SAVINGS)
            print("Running: {}".format(self.name.title()))
        elif self.name.upper() == "CHRISTOFIDES":
            print("Running: {}".format(self.name.title()))
            self.search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.CHRISTOFIDES)
        elif self.name.upper() == "PCA":
            print("Running: {}".format(self.name.title()))
            self.search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        elif self.name.upper() == "LCA":
            print("Running: {}".format(self.name.title()))
            self.search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.LOCAL_CHEAPEST_ARC)
        elif self.name.upper() == "GCA":
            print("Running: {}".format(self.name.title()))
            self.search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.GLOBAL_CHEAPEST_ARC)
        #endregion

        #region Optimization algorithms
        # self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC)
        # self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT)
        # self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        # self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING)
        # self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)
        # self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.OBJECTIVE_TABU_SEARCH)
        # self.search_parameters.time_limit.seconds = 10
        #endregion

        # Solve the problem.
        self.solution = self.routing.SolveWithParameters(self.search_parameters)

        # Print solution on console.
        if self.solution:
            self.print_solution(self.manager, self.routing, self.solution)
        
        self.routes = self.get_route(self.solution, self.routing, self.manager).pop()
        #endregion

    def distance_callback(self, from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.data['distance_matrix'][from_node][to_node]
    
    def print_solution(self, manager, routing, solution):
        """Prints solution on console."""
        # print("Objective: {}".format(solution.ObjectiveValue()))
        # print("-----------------------")
        # print('Objective: {} miles'.format(self.solution.ObjectiveValue()))
        index = routing.Start(0)
        plan_output = 'Route:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            # route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
            self._distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        print(plan_output)
        # print("Distance: {}".format(route_distance))
        self.end = time.time()
        self.calc_time = self.end - self.start
        print("{}: {}m, {}sec.".format(self.name, self._distance, round(self.calc_time, 3)))

    def get_route(self, solution, routing, manager):
        """Get vehicle routes from a solution and store them in a list."""
        routes = []
        for route_nbr in range(routing.vehicles()):
            index = routing.Start(route_nbr)
            route = [manager.IndexToNode(index)]
            while not routing.IsEnd(index):
                index = solution.Value(routing.NextVar(index))
                route.append(manager.IndexToNode(index))
            routes.append(route)
        return routes
    
    # def __repr__(self):
    #     output = repr(self.name)
    #     # output = repr(self.name)+"__"+repr(self._distance)+"__"+repr(round(self.calc_time, 3))
    #     return output