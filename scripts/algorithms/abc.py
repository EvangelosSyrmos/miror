#!/usr/bin/env python
from __future__ import print_function


class Abc(object):
    """
    User's Class structure and Necessary variables:
    ## Calculation Time
    ## Distance of the route
    ## List of Waypoints to
    """

    def __init__(self, distance_matrix):
        self.distance_matrix = distance_matrix

        self.calculation_time = 0.004   
        self._distance = 289           
        self.route = [0, 1, 2, 0]        
    