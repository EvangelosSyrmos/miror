#!/usr/bin/env python
from __future__ import print_function
from rospy.exceptions import ROSInitException, ROSInterruptException
from visualization_msgs.msg import Marker, MarkerArray
import rospy



class MarkerPoint(object):
    '''
    Marker Object with all the data 
    '''
    def __init__(self, x, y, id):
        '''
        Initialize all the values and Create 
        ROS Marker Obj
        '''
        #region User's values
        self.__x = x
        self.__y = y
        self.__id = id
        #endregion

        #region Default values
        self.__z = 0.5
        self.__type = 9 # Type 9 = Text View Facing
        self.__frame_id = "map"
        self.__scale_all_axis = 0.2
        self.__action = 0
        self.__color_a = 1.0
        self.__color_r = 1.0
        self.__color_g = 1.0
        self.__color_b = 0.0
        self.__orientation_w = 1.0
        #endregion

        #region Create Variable Marker
        self.marker_point = Marker()
        self.marker_point.header.frame_id = self.__frame_id
        self.marker_point.type = self.__type
        self.marker_point.action = self.__action
        self.marker_point.scale.x = self.__scale_all_axis
        self.marker_point.scale.y = self.__scale_all_axis
        self.marker_point.scale.z = self.__scale_all_axis
        self.marker_point.color.a = self.__color_a
        self.marker_point.color.r = self.__color_r
        self.marker_point.color.g = self.__color_g
        self.marker_point.color.b = self.__color_b
        self.marker_point.pose.position.x = self.__x
        self.marker_point.pose.position.y = self.__y
        self.marker_point.pose.position.z = self.__z
        self.marker_point.pose.orientation.w = self.__orientation_w
        self.marker_point.id = self.__id
        self.marker_point.text = str(self.__id)
        #endregion


    #region @Properties
    @property
    def x(self):
        return self.__x
    
    @property
    def y(self):
        return self.__y
    
    @property
    def z(self):
        return self.__z
    
    @property
    def type(self):
        return self.__type
    
    @property
    def frame_id(self):
        return self.__frame_id
    
    @property
    def scale_all_axis(self):
        return self.__scale_all_axis

    @property
    def action(self):
        return self.__action

    @property
    def color_a(self):
        return self.__color_a

    @property
    def color_r(self):
        return self.__color_r

    @property
    def color_g(self):
        return self.__color_g
    
    @property
    def color_b(self):
        return self.__color_b
    
    @property
    def orientation_w(self):
        return self.__orientation_w
    
    @property
    def id(self):
        return self.__id
    #endregion

    #region @Setters
    @x.setter
    def x(self, value):
        '''
        Setter for X (float)
        '''
        self.__x = float(value)
    
    @y.setter
    def y(self, value):
        '''
        Setter for Y (float)
        '''
        self.__y = float(value)
    
    @z.setter
    def z(self, value):
        '''
        Setter for Z (float)
        '''
        self.__z = float(value)
    
    @type.setter
    def type(self, value):
        '''
        Setter for Type (int):
        0 = ARROW,
        1 = CUBE,
        2 = SPHERE,
        3 = CYLINDER,
        4 = LINE_STRIP,
        5 = LINE_LIST,
        6 = CUBE_LIST,
        7 = SPHERE_LIST,
        8 = POINTS,
        9 = TEXT_VIEW_FACING,
        10 = MESH_RESOURCE,
        11 = TRIANGLE_LIST,
        '''
        self.__type = int(value)
    
    @frame_id.setter
    def frame_id(self, value):
        '''
        Setter for Frame ID
        '''
        self.__frame_id = value
    
    @scale_all_axis.setter
    def scale_all_axis(self, value):
        '''
        Scale Marker in all axis 0.0<(float)<1.0
        '''
        self.__scale_all_axis = float(value)
    
    @action.setter
    def action(self, value):
        '''
        0 = ADD
        1 = MODIFY
        2 = DELETE
        3 = DELETEALL
        '''
        self.__action = value
    
    @color_a.setter
    def color_a(self, value):
        '''
        Setter for Trensparency 0.0<(float)<1.0
        '''
        self.__color_a = float(value)
    
    @color_r.setter
    def color_r(self, value):
        '''
        Setter for RED Value 0.0<(float)<1.0
        '''
        self.__color_r = float(value)

    @color_g.setter
    def color_g(self, value):
        '''
        Setter for GREEN Value 0.0<(float)<1.0
        '''
        self.__color_g = float(value)

    @color_b.setter
    def color_b(self, value):
        '''
        Setter for BLUE Value 0.0<(float)<1.0
        '''
        self.__color_b = float(value)

    @orientation_w.setter
    def orientation_w(self, value):
        '''
        Setter for Orientation 0.0<(float)<1.0
        '''
        self.__orientation_w = float(value)
    
    @id.setter
    def id(self, value):
        '''
        Setter for ID Value
        '''
        self.__id = int(value)
    #endregion

    def __repr__(self):
        '''
        Pretify Object
        '''
        output = "Marker._X=["+repr(round(self.__x,4))+"]._Y=["+repr(round(self.__y,4))+"].ID=["+repr(self.__id)+"]"
        return output
