# MIROR - Mobile Industrial Robots with Optimal Routing algorithms
![Screenshot_1](https://user-images.githubusercontent.com/38979158/129700367-97a27572-2013-4730-ad05-e434afdcd792.png)

This application was developed ion order to create a sophisticated testing environment that applies routing optimization algorithms that solve the Travelling Salesman Problem to industial facility layouts, compares the results of the algorithmic outcomes and informs the effective and efficient routing solution. It is integrated with ROS, a robotic framework that provides tools such as Rviz for visualization, Gazebo 3D a simulation physics engine and Google OR an optimization suite that provides build-in algorithms that solve complex problems.

## The software tool provides:

1<sup>st</sup> Import and test custom algorithms.

2<sup>nd</sup> Evaluate the efficiency of the algorithms.

3<sup>rd</sup> Test selected/imported algorithms in any given layout.


## Instructions
1<sup>st</sup> ROS Master Node:
```bash
roscore
```
2<sup>nd</sup> Gazebo simulation:
```bash
roslaunch turtlebot3_gazebo tsp_world.launch
```
3<sup>rd</sup> ROS Navigation, with some changes:
```bash
roslaunch research tsp_navigation.launch
```
4<sup>th</sup> TSP GUI:
```bash
roslaunch research tsp_gui.launch
```

## Waypoint insertion and cost martix calculation
The stepwise process to create the waypoints and insert them in any given layout are shown in the GIF as follows.

NOTE: ***The first waypoint must be on the starting position of the robot (as a depot = starting / ending) node.***

1<sup>st</sup> Insert the number of waypoints in the given field and press the button ***Create Waypoints***.

2<sup>nd</sup> Insert the waypoints in Rviz with the button ***Publish Point*** inside the map.

3<sup>rd</sup> Once the waytpoints are placed press the button ***Calculate Costs*** to create the essential cost matrix for the algorithms.

![Waypoint_animation](https://user-images.githubusercontent.com/38979158/129730096-f6ea4794-b4c2-48f4-9736-d1cfe94d79e2.gif)

## Select any number of the provided algorithms to be executed sequentialy.

![tsp_gui](https://user-images.githubusercontent.com/38979158/117545808-0db7cc00-b030-11eb-85f6-eb2549e97779.png)

## Evaluate the results with the provided matplotlib window.

![Matplotlib](https://user-images.githubusercontent.com/38979158/117545829-2aec9a80-b030-11eb-8644-37dfa2e00883.png)

## Conventions for custom algorithm
1<sup>st</sup> In order to add an algorithm the python file must be created in the ***scripts/algorithm*** folder.

2<sup>nd</sup> The file name must be identical to the class inside it, e.g. ***abc.py***.

3<sup>rd</sup> The class must have a route list to store the waypoints, calculation time and route distance.

```python
class Abc:
    def __init__(self, distance_matrix):
        self.distance_matrix = distance_matrix
        self.calculation_time = 4.5     # Float
        self._distance = 290            # Float/Int
        self.route = []                 # [0, 1, 2, 0] 

```

## Route navigation
When the calculation of the selected/imported algorithms are done, the mobile robot can start executing the best algorithm with the lowest distance route by pressing the button ***Navigate/Move Robot***. If multiple algorithms have the same distances, the algorithm with the lowest calculation time will be selected and in an unlikely event of all route distances are same as well as calculation times a random algorithms will be selected.

NOTE: ***The user can execute multiple combinations of algorithms without the need of calculatin the cost matrix. Selecting different algorithms or importing custom can be done without the need to navigate the mobile robot.***

The following GIF shows the route of the best algorithm in the given test.
![ezgif com-gif-maker](https://user-images.githubusercontent.com/38979158/129703013-03c32680-cdc0-4cb7-9312-12a727b8535f.gif)


## Additional Information 
Local search strategies (Metaheuristics) known as optimizing algorihtms are enabled to run for 5 sec after the first solution has been given located in ***scripts/algorithms/google.py***. Feel free to change the duration or option for large scale problems.

```python
self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
#self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.AUTOMATIC)
#self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.GREEDY_DESCENT)
#self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.SIMULATED_ANNEALING)
#self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.TABU_SEARCH)
#self.search_parameters.local_search_metaheuristic = (routing_enums_pb2.LocalSearchMetaheuristic.OBJECTIVE_TABU_SEARCH)
self.search_parameters.time_limit.seconds = 5

```
## MIT License
[![License](https://img.shields.io/badge/license-MIT-green)](./LICENSE)
