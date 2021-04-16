#!/usr/bin/env python
from __future__ import print_function
from actionlib_msgs.msg import *
from move_base_msg.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import Point, PoseWithCovarianceStamped
import actionlib
import rospy


class Base():
    def __init__(self):
        self.__server = "move_base"
        self.__frame_id = "map"
        self.__amcl = "amcl_pose"
        self.__amcl = rospy.Subscriber(self.__amcl, PoseWithCovarianceStamped, self.__amcl_cb)
        self._connect()

    @staticmethod
    def __frame_id(self, frame: str):
        self.__frame_id = frame
        
    @staticmethod
    def __server(self, server: str):
        self.__server = server

    def _connect(self):
        ''' Connect with Move Base action Server '''
        self._ac = actionlib.SimpleActionClient(self.__server, MoveBaseAction)
        while not self._ac.wait_for_server(rospy.Duration.from_sec(5.0)):
            rospy.loginfo("Waiting for Move Base Server...")

    def __amcl_cb(self, data):
        ''' Updates pose '''
        self.x_pose
        self.y_pose

    def goal(self, x:float, y: float, z: float):
        ''' Send to goal '''

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = self.__frame_id
        goal.target_pose.header.stamp = rospy.Time.now()

        goal.target_pose.pose.position = Point(x, y, z)
        goal.target_pose.pose.orientation.x = 0.0
        goal.target_pose.pose.orientation.y = 0.0
        goal.target_pose.pose.orientation.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0

        self._ac.send_goal(goal) # Send to goal
        
        # region Waii for execution
        if self._ac.get_state() == GoalStatus.SUCCEEDED:
            return True
        else:
            return False


