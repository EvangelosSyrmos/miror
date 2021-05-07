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
import time
import importlib
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
        global imported_class

        #region Write empty file of aglos
        path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' # Find the ROS Package Path
        os.chdir(path) # Change directory
        with open('run_algos.txt', 'w') as f:
            f.write("")
        #endregion

        self.pressed_wp = False
        self.pressed_algo = False
        self.import_once = True
        self.num_of_wp = self.ids.waypoints_field.text
        self.wp_info = self.ids.waypoint_info
        self.algo_info = self.ids.algo_info
        self.user_class = self.ids.user_class.text
        self.run_info = self.ids.run_info
        self.checkboxes = []

    def make_file(self, name):
        """ Write the import for the module """
        file_name = "__init__.py"
        self.import_once = False
        #region Save olf file imports
        with open(file_name, 'r') as f:
            self.old_file_imports = f.read()
        #endregion

        with open(file_name, 'a') as f:
            f.write('\nfrom {} import {}'.format('.'+ name.lower(), name.title()))
    
    def reset_init_file(self):
        """ Reset the file module at the start of the application"""
        file_name = "__init__.py"
        os.chdir('../..')
        os.chdir('algorithms')
        with open(file_name, 'w') as f:
            f.write(self.old_file_imports)

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
        global imported_class
        self.algo_info = self.ids.algo_info
        self.user_class = self.ids.user_class.text
        # self.reset_init_file() # Reset the Init file

        if not self.checkboxes and not self.user_class != None:
            
            #region Empty file
            path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' # Find the ROS Package Path
            os.chdir(path) # Change directory
            with open('run_algos.txt', 'w') as f:
                f.write("")
            #endregion
            self.algo_info.text='[color=#FF0000]Select or insert algorithm.[/color]'
        
        if self.user_class != None:
            #region Users Class input
            filepath = os.getcwd()
            folder = 'algorithms'
            os.chdir('../..')
            os.chdir('{}'.format(folder))
            if self.import_once:
                self.make_file(self.user_class) #import class in init.py file
            full_name = folder + "." + self.user_class
            os.chdir('..')
            module = importlib.import_module(full_name)
            the_class = getattr(module, self.user_class.title())

            #region Read disctance matrix
            print(os.getcwd())
            cur_path = os.getcwd() + '/utils' + '/reusables'
            os.chdir(cur_path)
            data = []
            with open("cost_matrix.csv") as f:
                reader = csv.reader(f)
                data = list(reader)
            #endregion
            imported_class = the_class(data) # Create object
            #endregion

        if self.checkboxes:
            #region Fill file
            path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' # Find the ROS Package Path
            os.chdir(path) # Change directory
            with open('run_algos.txt', 'w') as f:
                for elem in self.checkboxes:
                    f.write("{}\n".format(elem))
            #endregion
            
            self.pressed_algo = True
            print("Running algorithms,", self.checkboxes)
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
        if self.user_class != None:
            temp = self.user_class
            name_list.append(temp.upper())
            name_list = [name.encode('utf-8') for name in name_list]
            temp = imported_class.calculation_time
            time_list.append(temp)
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
        global wp 
        wp = WaypointWindow()
        return wp

    def on_stop(self):
        """ Clear the init file when app stops """
        global wp
        wp.reset_init_file()

if __name__ == '__main__':
    rospy.init_node('gui_node')
    tsp = TspGuiApp()
    tsp.run()
    rospy.spin()
    