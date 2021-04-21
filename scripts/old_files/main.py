#!/usr/bin/env python
from __future__ import print_function
from utils.config_class import Config
from utils.waypoints_class import Waypoints
import utils
import rospy


def main():
    ''' 
    Initialize ROS Node, Parse values from Config
    Starts Rviz window
    '''
    cfg = utils.Config()
    rospy.init_node(cfg.ROS_NODE_NAME)
    pts = utils.Waypoints(3)
    rospy.spin()

if __name__ == '__main__':
    main()


