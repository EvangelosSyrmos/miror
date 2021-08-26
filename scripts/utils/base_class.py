#!/usr/bin/env python
from __future__ import print_function
from actionlib_msgs.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import actionlib
import rospy
import rospkg
import os

class MoveBase():
    def __init__(self, routes):

        #region MoveBase
        self.__server = "move_base"
        self.__frame_id = "map"
        self.__amcl = "amcl_pose"
        self.__amcl = rospy.Subscriber(self.__amcl, PoseWithCovarianceStamped, self.__amcl_cb)
        self._connect()
        #endregion

        #region Start moving robot
        self.routes = routes
        self.waypoints = self.get_waypoints()
        #endregion
        
        self.start_robot()

    def start_robot(self):
        """Move Robot in the Route list"""
        for route in self.routes[1:]:
            for waypoint in self.waypoints:
                if route == waypoint[0]:
                    status = self.goal(float(waypoint[1]), float(waypoint[2]))
        if status:
            return True
        else:
            return False
    
    def _connect(self):
        ''' Connect with Move Base action Server '''
        try:
            self._ac = actionlib.SimpleActionClient(self.__server, MoveBaseAction)
            while not self._ac.wait_for_server(rospy.Duration.from_sec(5.0)):
                rospy.loginfo("Waiting for Move Base Server...")
        except Exception as e:
            print(e)

    def __amcl_cb(self, data):
        ''' Updates pose '''
        self.x_pose
        self.y_pose

    def goal(self, x, y):
        ''' Send to goal '''
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.__frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(x, y, 0)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self._ac.send_goal(goal) # Send to goal
        self._ac.wait_for_result(rospy.Duration(60))

        # region Waii for execution
        if self._ac.get_state() == GoalStatus.SUCCEEDED:
            return True
        else:
            return False

    def get_waypoints(self):
        """Collect all the waypoint data from file"""
        try:
            path = rospkg.RosPack().get_path('miror')+'/scripts/utils/reusables' # Find the ROS Package Path
            os.chdir(path)
            with open('waypoints_information.txt', 'r') as file:
                l = [[float(num) for num in line.split(',')] for line in file]
        except Exception as e:
            print(e)
        return l 
