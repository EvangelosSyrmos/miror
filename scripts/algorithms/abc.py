#!/usr/bin/env python
from __future__ import print_function



class Abc(object):
    def __init__(self, distance_matrix):
        self.distance_matrix = distance_matrix
        self.calculation_time = 0.004