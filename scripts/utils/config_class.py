#!/usr/bin/env python
from __future__ import print_function
import configparser
import os


class Config(object):
    ''' Parse the args and values '''
    def __init__(self):
        config = configparser.ConfigParser()
        config.read(os.path.dirname(os.path.abspath(__file__)) + '/reusables/config.ini')

        #region ROS
        self.ROS_NODE_NAME = str(config['ROS']['ROS_NODE_NAME'])
        #endregion

        #region Algorithm
        self.MAXIMUM_MARKERS = int(config['ALGORITHM']['MAXIMUM_MARKERS'])
        #endregion

        #region Marker
        self.A = float(config['MARKER']['A'])
        self.R = float(config['MARKER']['R'])
        self.G = float(config['MARKER']['G'])
        self.B = float(config['MARKER']['B'])
        self.AXIS_Z = float(config['MARKER']['AXIS_Z'])
        self.SCALE = float(config['MARKER']['SCALE'])
        self.TYPE = float(config['MARKER']['TYPE'])
        #endregion
