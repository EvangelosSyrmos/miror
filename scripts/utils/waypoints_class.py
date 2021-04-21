#!/usr/bin/env python
from __future__ import print_function
from time import sleep
from rospy.exceptions import ROSInitException, ROSInterruptException
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from .config_class import Config
import nav_msgs.srv
import math
from std_msgs.msg import Float32, Header
from geometry_msgs.msg import PoseStamped, Point, Pose, Quaternion
from rospy.exceptions import ROSException
import rospy
import rospkg
import csv
import os


class Waypoints(object):
    '''
    This class is responsible for creating a ROS topic connection with 
    Visualize Markers from Rviz. The user can click on rviz to add all 
    the desired waypoints, thus the class can calculate the path cost
    by calling the ROS MakePlan Service. Saves the cost matrix in CSV
    for future uses.
    '''
    def __init__(self, number_of_waypoints):
        cfg = Config() # Parse all parameters

        #region ROS_TOPIC 
        self.__clicked_topic = '/clicked_point'
        self.__marker_topic = '/visualization_marker_array'
        self._sub_click_point = rospy.Subscriber(self.__clicked_topic, PointStamped, self.click_point_cb)
        self._pub_markers = rospy.Publisher(self.__marker_topic, MarkerArray, queue_size=10)
        # endregion

        #region Marker Info
        self.__marker_z = cfg.AXIS_Z
        self.__marker_type = 9
        self.__marker_frame_id = "map"
        self.__marker_scale_all_axis = 0.5
        self.__marker_id = 0
        self.__marker_action = 0
        self.__marker_color_a = cfg.A
        self.__marker_color_r = cfg.R
        self.__marker_color_g = cfg.G
        self.__marker_color_b = cfg.B
        self.__marker_orientation_w = 1.0
        self.MARKER_MAX = number_of_waypoints
        self.marker_array = MarkerArray()
        #endregion
        
    def save_to_file(self):
        '''
        Save Waypoints : ID, X, Y
        '''
        path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' # Find the ROS Package Path
        os.chdir(path) # Change directory

        #region Save in TXT {ID, X, Y} 
        with open("waypoints_information.txt", "w") as f:
            for mr in self.marker_array.markers:
                f.write("{},{},{}\n".format(mr.id, mr.pose.position.x, mr.pose.position.y))
        #endregion   

    def click_point_cb(self, data):
        '''
        Adds the current pose of the clicked point in 
        two variables to publish next
        '''
        
        x = float(data.point.x)
        y = float(data.point.y)

        #region MarkerArray Exceed Remove Oldest Marker
        if (len(self.marker_array.markers) >= self.MARKER_MAX):
            self.marker_array.markers.pop(0)
            id = 0
            for m in self.marker_array.markers:
                m.text = str(id)
                m.id = id
                id += 1
            self.__marker_id = id
            self._pub_markers.publish(self.marker_array)
        #endregion

        marker = self.create_marker(x, y) # Create a marker
        self.marker_array.markers.append(marker) # Add new Marker
        self._pub_markers.publish(self.marker_array)
        self.__marker_id += 1
  
    def create_marker(self, x, y):
        '''
        Create a marker of type Marker
        '''
        marker = Marker()
        marker.header.frame_id = self.__marker_frame_id
        marker.type = self.__marker_type
        marker.action = self.__marker_action
        marker.scale.x = self.__marker_scale_all_axis
        marker.scale.y = self.__marker_scale_all_axis
        marker.scale.z = self.__marker_scale_all_axis
        marker.color.a = self.__marker_color_a
        marker.color.r = self.__marker_color_r
        marker.color.g = self.__marker_color_g
        marker.color.b = self.__marker_color_b
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = self.__marker_z
        marker.pose.orientation.w = self.__marker_orientation_w
        marker.id = self.__marker_id

        if self.__marker_type == 9:
            marker.text = str(self.__marker_id)

        return marker

    def calculate_cost(self, poses):
        '''
        Calculates the euclidean distance from each consecutive point
        and returns the cost
        '''
        sum = 0
        for pose in poses:
            sum += math.sqrt(math.pow(((pose.pose.position.x + 1)- (pose.pose.position.x)),2)
                            + math.pow(((pose.pose.position.y + 1)- (pose.pose.position.y)),2))
        return sum

    def start_calculations(self):
        '''
        Connect to MakePlan service and save the path cost
        for each waypoint in nested lists
        '''
        #region Wait for Move Base Service to respond
        try:
            rospy.wait_for_service('/move_base/make_plan', timeout=5.0)
        except ROSException:
            print("Service not responding.")
        #endregion

        #region Create a connection with the service
        try:
            get_plan = rospy.ServiceProxy('/move_base/make_plan', nav_msgs.srv.GetPlan)
        except rospy.service.ServiceException as e:
            print("Service failed {}".format(e))
        #endregion

        #region Loop over each Waypoint to get the Cost
        cost_list = []
        for mr_start in self.marker_array.markers:
            inner_list = []
            for mr_goal in self.marker_array.markers:
                if int(mr_start.id) == int(mr_goal.id):
                    cost = 0.0
                    inner_list.append(cost)
                else:
                    start = PoseStamped()
                    start.header.frame_id = "map"
                    start.header.stamp = rospy.Time.now()
                    start.pose.position = Point(mr_start.pose.position.x, mr_start.pose.position.y, 0)

                    goal = PoseStamped()
                    goal.header.frame_id = "map"
                    goal.header.stamp = rospy.Time.now()
                    goal.pose.position = Point(mr_goal.pose.position.x, mr_goal.pose.position.y, 0)

                    tolerance = Float32()
                    tolerance = 0.2

                    # Create a Reqeust to the service
                    req = nav_msgs.srv.GetPlanRequest(start, goal, tolerance)

                    # Get the response with all the points
                    resp = get_plan(req)
                    # print("Poits {}".format(len(resp.plan.poses)))
                    
                    # Calculate path's total cost 
                    cost = self.calculate_cost(resp.plan.poses)
                    # print("Cost: {}".format(cost))
                    inner_list.append(cost)
            cost_list.append(inner_list)
        #endregion

        self.save_cost_matrix(cost_list) # Save the Cost Matrix in CSV 
    
    def save_cost_matrix(self, cost_list):
        '''
        Dump the cost matrix from all the Waypoints in order
        '''
        path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' # Find the ROS Package Path
        os.chdir(path) # Change directory

        #region Save in CSV format
        with open("cost_matrix.csv", "wb") as f:
            writer = csv.writer(f)
            writer.writerows(cost_list)
            print("Done writing")
        #endregion
