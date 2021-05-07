# Application for TSP algorithms
This application was created having a main purpose to integrate Waypoints inside ROS visualized in Rviz,to run Google OR Tools build in TSP algorithms and One by the user.

## Description
The user can type the number of waypoints the robot need to travel, but the first one has to be on the starting position of the robot (as a depot = starting / ending). Most TSP algorithms are included in the checkbox field. Optimizing algorihtms are also included in the gui.py in order to select which to run after the first solution has been given. The user can type the name of his algorithm inside the Text Input following some conventions.

## Conventions
1<sup>st</sup> In order to add an algorithm the python file must be created in the ***scripts/algorithm*** folder.

2<sup>nd</sup> The file name must be identical to the class inside it, e.g. ***pso.py***.

3<sup>rd</sup> The class must have a route list with a getter function implemented accordingly.

```python
class Pso(object):
    def __init__(self, distance_matrix):
        self.distance_matrix = distance_matrix
        self.route = []
        
    @property
    def route(self):
        return self.route
```
## Execution
ROS Master Node:
```bash
roscore
```
Gazebo simulation:
```bash
roslaunch turtlebot3_gazebot tsp_world.launch
```
ROS Navigation, with some changes:
```bash
roslaunch research tsp_navigation.launch
```
TSP GUI:
```bash
roslaunch research tsp_gui.launch
```
