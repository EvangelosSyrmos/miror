#!/usr/bin/env python
from  __future__ import print_function
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import sys

class Google(object):
    def __init__(self, distance_matrix):

        #region Typecasting distance matrix --> Int
        temp = []
        for row in distance_matrix:
            temp_inner = []
            for col in row:
                temp_inner.append(int(col))
            temp.append(temp_inner)
        #endregion

        #region Create Data model
        self.data = {}
        self.data['distance_matrix'] = temp
        # print(self.data['distance_matrix'])
        self.data['num_vehicles'] = 1
        self.data['depot'] = 0
        #endregion

        #Create routing index manager
        self.manager = pywrapcp.RoutingIndexManager(len(self.data['distance_matrix']), self.data['num_vehicles'], self.data['depot'])

        #Create routing Model
        self.routing = pywrapcp.RoutingModel(self.manager)

        self.transit_callback_index = self.routing.RegisterTransitCallback(self.distance_callback)

        #Define cost of each arc.
        self.routing.SetArcCostEvaluatorOfAllVehicles(self.transit_callback_index)

        # Setting first solution heuristic
        self.search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        self.search_parameters.first_solution_strategy = (routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)

        # Solve the problem.
        self.solution = self.routing.SolveWithParameters(self.search_parameters)

        # Print solution on console.
        if self.solution:
            self.print_solution(self.manager, self.routing, self.solution)
    
    def get_routes(self, solution, routing, manager):
        routes = []
        for route_nbr in range(routing.vehicles()):
            index = routing.Start(route_nbr)
            route = [manager.IndexToNode(index)]
            while not routing.IsEnd(index):
                index = solution.Value(routing.NextVar(index))
                route.append(manager.IndexToNode(index))
                routes.append(route)
        return routes

    def distance_callback(self, from_index, to_index):
        """Returns the distance between the two nodes."""
        # Convert from routing variable Index to distance matrix NodeIndex.
        from_node = self.manager.IndexToNode(from_index)
        to_node = self.manager.IndexToNode(to_index)
        return self.data['distance_matrix'][from_node][to_node]
    
    def print_solution(self, manager, routing, solution):
        """Prints solution on console."""
        print("Objective: {}".format(solution.ObjectiveValue()))
        print("-----")
        # print('Objective: {} miles'.format(self.solution.ObjectiveValue()))
        index = routing.Start(0)
        plan_output = 'Route for vehicle 0:\n'
        route_distance = 0
        while not routing.IsEnd(index):
            plan_output += ' {} ->'.format(manager.IndexToNode(index))
            previous_index = index
            index = solution.Value(routing.NextVar(index))
            route_distance += routing.GetArcCostForVehicle(previous_index, index, 0)
        plan_output += ' {}\n'.format(manager.IndexToNode(index))
        print(plan_output)
        plan_output += 'Route distance: {}miles\n'.format(route_distance)