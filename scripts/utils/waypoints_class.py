#!/usr/bin/env python
from __future__ import print_function
from time import sleep
from rospy.exceptions import ROSInitException, ROSInterruptException
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from .config_class import Config
import rospy


class Waypoints(object):
    def __init__(self):
        cfg = Config() # Parse all parameters

        #region ROS_TOPIC 
        self.__clicked_topic = '/clicked_point'
        self.__marker_topic = '/visualization_marker_array'
        self._sub_click_point = rospy.Subscriber(self.__clicked_topic, PointStamped, self.click_point_cb)
        self._pub_markers = rospy.Publisher(self.__marker_topic, MarkerArray, queue_size=10)
        # endregion

        #region Marker Info
        self.__marker_z = cfg.AXIS_Z
        self.__marker_type = cfg.TYPE 
        self.__marker_frame_id = "map"
        self.__marker_scale_all_axis = cfg.SCALE
        self.__marker_id = 0
        self.__marker_action = 0
        self.__marker_color_a = cfg.A
        self.__marker_color_r = cfg.R
        self.__marker_color_g = cfg.G
        self.__marker_color_b = cfg.B
        self.__marker_orientation_w = 1.0
        self.MARKER_MAX = cfg.MAXIMUM_MARKERS
        self.marker_array = MarkerArray()
        #endregion

        # self.delete_all_active_markers() # Delete all active markers
        

    # def delete_all_active_markers(self):
    #     '''
    #     Publishes action = 3 (DELETEALL) in marker topic,
    #     to delete all active markers in RVIZ
    #     '''
    #     delete_marker = Marker()
    #     delete_marker.action = 3
    #     self.marker_array.markers.append(delete_marker)
    #     self._pub_markers.publish(self.marker_array)

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
    
    def start_cacl(self):
        print("start calc")
        print("The length is", len(self.marker_array.markers))
        print(self.marker_array.markers)