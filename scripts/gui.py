#!/usr/bin/env python2.7
from __future__ import print_function
from logging import info
import kivy
import os
import sys
from kivy.core import text
import rospy
import rospkg
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.lang import Builder
from utils.waypoints_class import Waypoints

path = rospkg.RosPack().get_path('research')+'/scripts'+ '/TspGui.kv'
Builder.load_file(path)

class WaypointWindow(BoxLayout):
    def __init__(self, *args):
        super(WaypointWindow, self).__init__(*args)
        global wp
        self.pressed = False
        self.num_of_wp = self.ids.waypoints_field.text
        self.wp_info = self.ids.waypoint_info

    def start_placing_waypoints(self):
        global wp
        self.num_of_wp = self.ids.waypoints_field.text
        self.wp_info = self.ids.waypoint_info

        if self.num_of_wp == '':
            self.wp_info.text='[color=#FF0000]Insert a number to continue.[/color]'
        elif self.num_of_wp == '1':
            self.wp_info.text='[color=#FF0000]Number has to be > 1.[/color]'
        else:
            temp_num = '[color=#50d0d9]'+str(self.num_of_wp)+'[/color]'
            self.wp_info.text='[color=#00FF00]Place [/color]'+temp_num+'[color=#00FF00] waypoints in Rviz.[/color]'
            self.pressed = True
            wp = Waypoints(int(self.num_of_wp)) # Create Wp Object

    def start_cost_matrix(self):
        global wp
        self.num_of_wp = self.ids.waypoints_field.text
        self.wp_info = self.ids.waypoint_info

        if self.pressed and self.num_of_wp != '':
            self.wp_info.text='[color=#71a381]Cost matrix created.[/color]'
            self.pressed = False
            wp.save_to_file() # Save all waypoints to CSV
            wp.start_calculations() # Create cost matrix
        else:
            self.wp_info.text='[color=#FF0000]Create Waypoints in Rviz first.[/color]'
            

class TspGuiApp(App):
    def build(self):
        return WaypointWindow()


if __name__ == '__main__':
    rospy.init_node('gui_node')
    tsp = TspGuiApp()
    tsp.run()
    rospy.spin()

    