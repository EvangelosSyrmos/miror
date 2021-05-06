#!/usr/bin/env python2.7
from __future__ import print_function
import imp
from importlib import import_module
from logging import info
from PIL.Image import ROTATE_270
import kivy
import os
import sys
import csv
from kivy.core import text
import rospy
import rospkg
from kivy.app import App
from kivy.uix.boxlayout import BoxLayout
from kivy.lang import Builder
import matplotlib.pyplot as plt
from utils.waypoints_class import Waypoints
from utils.base_class import MoveBase
from algorithms.google import Selector

path = rospkg.RosPack().get_path('research')+'/scripts'+ '/TspGui.kv'
Builder.load_file(path)

class WaypointWindow(BoxLayout):

    def __init__(self, *args):
        super(WaypointWindow, self).__init__(*args)
        global wp
        global gl 

        #region Write empty file of aglos
        path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' # Find the ROS Package Path
        os.chdir(path) # Change directory
        with open('run_algos.txt', 'w') as f:
            f.write("")
        #endregion

        self.pressed_wp = False
        self.pressed_algo = False
        self.num_of_wp = self.ids.waypoints_field.text
        self.wp_info = self.ids.waypoint_info
        self.algo_info = self.ids.algo_info
        self.run_info = self.ids.run_info
        self.checkboxes = []
    
    def checkbox_click(self, instance, value, algo):
        '''
        Updates the list of algorithms to run
        '''
        self.algo_info = self.ids.algo_info
        if value:
            self.checkboxes.append(algo)
        else:
            self.checkboxes.remove(algo)
        output = ', '.join([str(elem) for elem in self.checkboxes])
        if not self.checkboxes:
            self.algo_info.text=''
        else:
            temp = '[color=#00FF00]'+str(output)+ '[/color]'
            self.algo_info.text = '[color=#50d0d9]Selected: [/color]'+ temp

    def run_algorithms(self):
        ''' 
        Run all selected algorithms, or users
        '''
        global gl
        global wp
        self.algo_info = self.ids.algo_info

        if not self.checkboxes:
            #region Empty file
            path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' # Find the ROS Package Path
            os.chdir(path) # Change directory
            with open('run_algos.txt', 'w') as f:
                f.write("")
            #endregion
            self.algo_info.text='[color=#FF0000]Select or insert algorithm.[/color]'
        else:
            #region Fill file
            path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' # Find the ROS Package Path
            os.chdir(path) # Change directory
            with open('run_algos.txt', 'w') as f:
                for elem in self.checkboxes:
                    f.write("{}\n".format(elem))
            #endregion
            
            self.pressed_algo = True
            print("Running algorithms,", self.checkboxes)
            # try:
            #     gl = Google(wp.get_cost_list())
            # except:
            os.chdir('..')
            cur_path = os.getcwd()
            os.chdir(cur_path + '/reusables')
            data = []
            with open("cost_matrix.csv") as f:
                reader = csv.reader(f)
                data = list(reader)
            gl = Selector(data, self.checkboxes)
            
            #region Matplotlib window
            name_list = [name.name for name in gl.alg_list]
            time_list = [name.calc_time for name in gl.alg_list]
            plt.barh(name_list, time_list)
            plt.grid(True)
            plt.xlabel("Seconds")
            plt.title("Algorithm results")
            plt.xticks(rotation = 45)
            plt.yticks(rotation = 45)
            plt.show()
            #endregion

    def move_robot(self):
        """Call MoveBase to start moving robot to the goal """
        global gl
        global mb
        self.run_info = self.ids.run_info
        if self.pressed_algo:
            self.run_info.text='[color=#0010FF]Route completed.[/color]'
            mb = MoveBase(gl.get_best_path())
            # mb = MoveBase(gl.routes)
        else:
            self.run_info.text='[color=#FF0000]Run algorithms first.[/color]'

    def start_placing_waypoints(self):
        global wp
        self.num_of_wp = self.ids.waypoints_field.text
        self.wp_info = self.ids.waypoint_info

        if self.num_of_wp == '':
            self.wp_info.text='[color=#FF0000]Insert a number to continue.[/color]'
        elif self.num_of_wp == '1':
            self.wp_info.text='[color=#FF0000]Number has to be > 1.[/color]'
        else:
            temp_num = '[color=#101000]'+str(self.num_of_wp)+'[/color]'
            self.wp_info.text='[color=#0010FF]Place [/color]'+temp_num+'[color=#0010FF] waypoints in Rviz.[/color]'
            self.pressed_wp= True
            wp = Waypoints(int(self.num_of_wp)) # Create Wp Object

    def start_cost_matrix(self):
        global wp
        self.num_of_wp = self.ids.waypoints_field.text
        self.wp_info = self.ids.waypoint_info
        if self.pressed_wp and self.num_of_wp != '':
            self.wp_info.text='[color=#71a381]Cost matrix created.[/color]'
            self.pressed_wp = False
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
    