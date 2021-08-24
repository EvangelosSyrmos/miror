# MIROR - Mobile Industrial Robots with Optimal Routing algorithms
![Landing Photo](https://user-images.githubusercontent.com/38979158/130687780-37381a41-d802-46c6-95ef-2d457f621d88.jpg)



This application was developed in order to create a sophisticated testing environment that applies routing optimization algorithms that solve the Travelling Salesman Problem to industial facility layouts, compares the results of the algorithmic outcomes and informs the effective and efficient routing solution. 
It is integrated with ROS, a robotic framework that provides tools such as Rviz for visualization, Gazebo 3D a simulation physics engine and Google OR an optimization suite that provides build-in algorithms that solve complex problems.

## The software tool provides:

- Import and test custom algorithms for TSP-[Travelling Salesman Problem](https://en.wikipedia.org/wiki/Travelling_salesman_problem).

- Evaluate the efficiency of the algorithms with graphs.

- Test selected/imported algorithms in any given layout.

## Prerequisites 
- [Ubuntu 18.04 LTS](https://releases.ubuntu.com/18.04.5/)
- [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
- [Turtlebot3](https://github.com/ROBOTIS-GIT/turtlebot3)
- [Kivy v1.9.1](https://pypi.org/project/Kivy/1.9.1/)
- [Matplotlib](https://pypi.org/project/matplotlib/)


## Installation
Creat a ROS workspace for all the corresponding packages: [catkin_ws](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)

Clone the latest version into your catkin workspace and compile the package using ROS.

```bash
cd catkin_ws/src
git clone git@github.com:EvangelosSyrmos/research.git
```
**Move the `turtlebot3_gazebo` folder provided by MIROR in the `catkin_ws/src`.**

To build the workspace and the type:
```bash
cd ..
catkin_make
```

## Run MIROR

Start ROS Master node

```bash
roscore
```

Start Gazebo simulation

```bash
roslaunch turtlebot3_gazebo tsp_world.launch
```

Start ROS Navigation launch file

```bash
roslaunch research tsp_navigation.launch
```

Start MIROR:

```bash
roslaunch research tsp_gui.launch
```


## Demo - Example 

The following examples use the already provided 3D map in Gazebo.

### Create and insert waypoints
To insert waypoints in Rviz, the steps are as follows:
- Enter the number of waypoints in the placeholder as shown.
![Number of waypoints](https://user-images.githubusercontent.com/38979158/130687972-c3069c3b-7450-4b4b-a723-641db6331f2a.jpg)
- Press the button ***Create waypoints*** and click the Rviz button ***Publish Point*** as shown for every point **starting from the mobile robot**.
![Number of waypoints in Rviz](https://user-images.githubusercontent.com/38979158/130677177-69d0c108-2b39-4093-bc13-cf03b79f5d8f.jpg)
- Finally press the button ***Calculate costs*** to create a NxN cost matrix for all the possible routes.
![Calculate costs](https://user-images.githubusercontent.com/38979158/130688056-effb3d21-2119-4f9a-99ce-5e5199e911b5.jpg)

### Import and execute custom algorithms
To import a custom algorithm and assess the performance in comparison to the provided some conventions must be followed.
#### Conventions:
- Create the python file in the specific directory `catkin_ws/src/research/scripts/algorithm/` folder.
- The python file name must be identical to the class inside, e.g. ***abc.py***.
- The class must have a route list to store the waypoints, calculation time and route distance.
```python
class Abc:
    def __init__(self, distance_matrix):
        self.distance_matrix = distance_matrix
        self.calculation_time = 4.5     # Float
        self._distance = 290            # Float/Int
        self.route = []                 # [0, 1, 2, 0] 

```
#### Execute custom algorithm
- Type the name of the `python` file in the placeholder as shown.
![Custom_algorithm](https://user-images.githubusercontent.com/38979158/130679903-41e42553-b065-4a64-881d-22ff2bb382fb.jpg)
- Press the button ***Execute*** to run the custom algorithm and any selected algorithm from the list sequentialy.

### Execute Google OR algorihtms
To execute any combination of the provided algorithms.
- Select them by clicking on the checkboxes as shown. An indicator for the current selection of algorithms is provided for feedback.
![Select algorithms](https://user-images.githubusercontent.com/38979158/130688147-50e70af8-6dc3-4694-87ad-6838ceea2bbc.jpg)
- Press the button ***Execute*** to run them sequentialy.
![Execute algorithms](https://user-images.githubusercontent.com/38979158/130680270-96692b82-b9ef-452b-b966-6e29cb95f47d.jpg)
-  A window with the results will pop with graphs.
![Graphs](https://user-images.githubusercontent.com/38979158/130678516-f50e4aa8-ca1f-43df-a576-81cfdbe7149d.jpg)

### Move mobile robot 
After the results have been shown the best algorithm will be selected based on the smallest route distance. If all the solutions are equal the algorithm with the fastes computation time will
selected.
- Press the button ***Navigate*** to start the navigation of the mobile robot in the facility layout as shown.
![Navigation](https://user-images.githubusercontent.com/38979158/130680972-d3079d0a-5956-4eb9-ab60-23e26d07cd73.jpg)
- The mobile robot will start moving with the best solution.
![navigation_GIF](https://user-images.githubusercontent.com/38979158/130684313-0fd0054a-7391-468c-9273-7768a4b65486.gif)

### NOTE
- ***The user can execute multiple combinations of algorithms without the need of calculating the cost matrix every time.***

- ***Selecting different algorithms or importing custom can be done without the need to navigate the mobile robot.***

### Additional Information
#### Import custom world map
To import a custom map follow the instructions [GazeboSim](http://gazebosim.org/tutorials?tut=ros_roslaunch).
- Use **turtlebot3_slam** to generate the occupancy grid [Turtlebot3_Slam](https://emanual.robotis.com/docs/en/platform/turtlebot3/slam/#run-slam-node).

- Save the generated maps.
```bash
rosrun map_server map_saver -f map
```
- Move the generated files `map.pgm` and `map.yaml` to `research/maps` and rename the old ones.

#### Local Search Metaheuristics
Local search strategies (Metaheuristics) known as optimizing algorihtms are enabled to run for 5 sec after the first solution has been given located in ***scripts/algorithms/google.py***.
Feel free to change the duration or option for large scale problems if needed.
```python
self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
# self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC)
# self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT)
# self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING)
# self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)
# self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.OBJECTIVE_TABU_SEARCH)
self.search_parameters.time_limit.seconds = 5
```
## Authors

- [@EvangelosSyrmos](https://github.com/EvangelosSyrmos)

  
## Support

For support, email me at evangelossyrmos@gmail.com

  
## License
[MIT](https://choosealicense.com/licenses/mit/)

  