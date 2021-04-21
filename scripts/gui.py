#!/usr/bin/env python2.7
import kivy
import os
from kivy.core import text
import rospy
import rospkg
from kivy.app import App
from kivy.uix.label import Label
from kivy.uix.gridlayout import GridLayout
from kivy.uix.textinput import TextInput
from kivy.uix.button import Button
from utils.waypoints_class import Waypoints

class StartingPage(GridLayout):
    def __init__(self, *args):
        global wp
        global th
        super(StartingPage, self).__init__(*args)
        self.cols = 2
        
        #region Load previous values from file if exists, else pass
        path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' 
        os.chdir(path) # Change directory
        if os.path.isfile("gui_prev_values.txt"):
            with open("gui_prev_values.txt", "r") as f:
                d = f.read().split(",")
                prev_num_of_wpoints = d[0]
        else:
            prev_num_of_wpoints = ""
        #endregion

        #region Number of waypoints
        self.add_widget(Label(text='Number of waypoints'))
        self.number_of_waypoints = TextInput(text=prev_num_of_wpoints)
        self.add_widget(self.number_of_waypoints)
        #endregion

        #region Create Waypoints
        self.param_button = Button(text="Submit Parameters")
        self.param_button.bind(on_press = self.create_waypoints)
        self.add_widget(self.param_button)
        #endregion

        #region Save Waypoints
        self.save_button = Button(text="Save Waypoints")
        self.save_button.bind(on_press = self.save_waypoints)
        self.add_widget(self.save_button)
        #endregion

    def create_waypoints(self, instance):
        '''
        Create instance of Waypoints
        '''
        global wp 
        print("Creating Wp)")
        try:
            wp = Waypoints(int(self.number_of_waypoints.text))
        except ValueError as e:
            print(str(e))

    def save_waypoints(self, instance):
        '''
        Save the old values for GUI and save Waypoints to CSV
        '''
        print("In save button")
        global wp
        global th
        num_of_waypoints = self.number_of_waypoints.text

        #region Load previous inputs
        path = rospkg.RosPack().get_path('research')+'/scripts/utils/reusables' 
        os.chdir(path) # Change directory
        with open("gui_prev_values.txt", "w") as f:
                f.write("{}".format(num_of_waypoints))
        #endregion

        wp.save_to_file() # Save all waypoints to CSV
        wp.start_calculations()


class GuiApp(App):
    def build(self):
        return StartingPage()


if __name__ == '__main__':
    rospy.init_node("gui_node")
    GuiApp().run()
    rospy.spin()