# A Multi-Agent Path Planing Algorithm Based on Velocity Obstacles
Welcome!
This multi-agent path planning algorithm is based on a reactive approch to avoid collisions with other robots in the run time. This algorithm is an attempt to implement 'Velocity Obstacles' or 'VO' without the information of other agents' velocity or position.
For details please click [here](). 

<!-- TOC START min:1 max:5 link:true asterisk:false update:true -->
- [Multi-Agent Algorithm Based on VO](#A-Multi-Agent-Path-Planing-Algorithm-Based-on-Velocity-Obstacles)
  - [System](#system)
  - [Implementation](#implementation)
    - [Python Scripts Nodes and Topics](#python-scripts-nodes-and-topics)
      - [PlainGround_Formation](#PlainGround_Formation)
      - [Run_Algo_Multiprocessing](#Run_Algo_Multiprocessing)
      - [VO_Based_Algo_Version2](#VO_Based_Algo_Version2)
  - [Execution](#execution)
  - [Results](#results)
      
<!-- TOC END -->

## System
This framework was tested under:
- Ubuntu 18.04
- ROS Melodic
- Gazebo
- Python 2.7

## Implementation
The Algorithm is being tested on turtlebot3(BURGER) in Gazebo simulation. 
The following Figure illustrates the node graph for one robot
(captured in RQT). 

![alt text](res/imgs/graphs/node_graph.png "node_graph")

### Python Scripts, Nodes and Topics

#### PlainGround_Formation
This python program will be resposible to run 'CircleFormationGoals' node and will publish this information using the topic(/goal), through a custom msg(Information).
(The Purpose of aligning robots in circle formation and to make them swap positions with the robots on the opposite side, is to make sure maximum interaction among the moving robots and hence to see the efficeincy of the algorithm )

#### Run_Algo_Multiprocessing
The **/run_algo_multiprocessing** is reponsible for runnning the main algorithm as many times as number of robots given by the user, using **/Python_Multiprocessing** feature. Moreover this program will subscribe to the topic(/goal) and distribute it among the robots.

#### VO_Based_Algo_Version2
This is the main program with the algorithm. This will initiate the node with the name of the agent and subscribes to the topics **/tb3_x/odom** and **/tb3_x/scan**. After the processing, the node is responsiblefor controlling the robots through publishing **/tb3_x/cmd_vel**.

## Execution
Below are the instructions in order to run the algorithm.

First, make sure you are in the **/Based_on_VO** branch and **/rvo_turtlebot3** package.
The following sequence is crucial and must be followed as given;

1. Before running the following command make sure that you have **/turtlebot3_gazebo** package installed in your PC, otherwise the simulation enviornment will not work. This command will run the simulation enviornment, spawn the robot models(4,8 or 16) and will start the static transform publisher (TF)

```
roslaunch rvo_turtlebot3 turtlebot3_empty_world_4agents.launch 
 or
roslaunch rvo_turtlebot3 turtlebot3_empty_world_8robots.launch 
 or
roslaunch rvo_turtlebot3 turtlebot3_empty_world_16robots.launch 
```
2. After the successful spawing of the robots, use the following command to create circle formation for testing;
   (The programm will ask for the number of robots, depending on how many robots have been spawned in the simulation kindly choose from 4,8 or 16)
```
~/driving_swarm/DrivingSwarm/src/pathplanning/vo_based_algo_without_benchmark/src$ python plainGround_formation.py 
```
3. Once the robots are aligned, run the algorithm using following command :
```
~/driving_swarm/DrivingSwarm/src/pathplanning/vo_based_algo_without_benchmark/src$ python run_algo_multiprocessing.py
```

### Results
...........................................


```
---------------------- BENCHMARK ----------------------
==> datetime: 2019-12-30 17:18:43.903142
==> Benchmark ID: 1577722723
==> model_name: turtlebot3
==> model_type: burger
==> namespace: tb3_
==> number_of_robots: 4
==> formation: dense_block
==> position: [1.5, 0.5, 0.5]
==> orientation: [0.0, 0.0, 0.0]
==> wp_map: tb3_edge
==> wp_threshold: 0.2
==> world: turtlebot3.world
==> rounds: 1
==> end_procedure: start
--------------------------------------------------------

[2019-12-30 17:20:41.077264] [WPTIME] tb3_3 from [1.8, 0.0, 0.0] to [-1.8, 0.0, 0.0] in 74.234s
[2019-12-30 17:20:47.376626] [WPTIME] tb3_1 from [1.8, 0.0, 0.0] to [-1.8, 0.0, 0.0] in 76.238s
[2019-12-30 17:21:36.520303] [WPTIME] tb3_0 from [1.8, 0.0, 0.0] to [-1.8, 0.0, 0.0] in 92.34s
[2019-12-30 17:21:49.072862] [WPTIME] tb3_2 from [1.8, 0.0, 0.0] to [-1.8, 0.0, 0.0] in 96.356s
[2019-12-30 17:21:49.108248] [WPTIME] tb3_3 from [-1.8, 0.0, 0.0] to [0.0, 1.8, 0.0] in 22.136s
[2019-12-30 17:22:39.036313] [WPTIME] tb3_0 from [-1.8, 0.0, 0.0] to [0.0, 1.8, 0.0] in 20.118s
[2019-12-30 17:22:39.066147] [WPTIME] tb3_1 from [-1.8, 0.0, 0.0] to [0.0, 1.8, 0.0] in 36.224s
[2019-12-30 17:22:51.752975] [WPTIME] tb3_3 from [0.0, 1.8, 0.0] to [0.0, -1.8, 0.0] in 20.11s
[2019-12-30 17:23:10.841155] [WPTIME] tb3_2 from [-1.8, 0.0, 0.0] to [0.0, 1.8, 0.0] in 26.162s
[2019-12-30 17:23:30.159001] [WPTIME] tb3_1 from [0.0, 1.8, 0.0] to [0.0, -1.8, 0.0] in 16.086s
[2019-12-30 17:23:42.918892] [WPTIME] tb3_0 from [0.0, 1.8, 0.0] to [0.0, -1.8, 0.0] in 20.112s
[2019-12-30 17:24:01.810786] [WPTIME] tb3_2 from [0.0, 1.8, 0.0] to [0.0, -1.8, 0.0] in 16.102s
[2019-12-30 17:24:01.850099] [WPTIME] tb3_3 from [0.0, -1.8, 0.0] to [1.8, 0.0, 0.0] in 22.146s
[2019-12-30 17:24:45.809623] [WPTIME] tb3_1 from [0.0, -1.8, 0.0] to [1.8, 0.0, 0.0] in 24.158s
[2019-12-30 17:24:45.841787] [WPTIME] tb3_3 from [1.8, 0.0, 0.0] to [-1.8, 0.0, 0.0] in 14.082s
[2019-12-30 17:24:45.842267] [MAKESPAN] 3 finished with makespan of 152.71s <<<
[2019-12-30 17:24:45.843561] [FLOWTIME] 3: 38.1775
[2019-12-30 17:24:58.359468] [WPTIME] tb3_0 from [0.0, -1.8, 0.0] to [1.8, 0.0, 0.0] in 24.156s
[2019-12-30 17:25:04.621223] [WPTIME] tb3_2 from [0.0, -1.8, 0.0] to [1.8, 0.0, 0.0] in 20.138s
[2019-12-30 17:25:29.290527] [WPTIME] tb3_1 from [1.8, 0.0, 0.0] to [-1.8, 0.0, 0.0] in 14.078s
[2019-12-30 17:25:29.291749] [MAKESPAN] 1 finished with makespan of 166.784s <<<
[2019-12-30 17:25:29.292400] [FLOWTIME] 1: 41.6965
[2019-12-30 17:25:47.299039] [WPTIME] tb3_2 from [1.8, 0.0, 0.0] to [-1.8, 0.0, 0.0] in 14.05s
[2019-12-30 17:25:47.300320] [MAKESPAN] 2 finished with makespan of 172.81s <<<
[2019-12-30 17:25:47.301209] [FLOWTIME] 2: 43.2025
[2019-12-30 17:26:28.273215] [WPTIME] tb3_0 from [1.8, 0.0, 0.0] to [-1.8, 0.0, 0.0] in 30.152s
[2019-12-30 17:26:28.275050] [MAKESPAN] 0 finished with makespan of 186.88s <<<
[2019-12-30 17:26:28.276239] [FLOWTIME] 0: 46.72
```
THANK YOU!
