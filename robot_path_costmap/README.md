# Path costmap (by layer injection)
Welcome to multi-robot path planning with adapted costmaps based on path plans of the other robots! 

Its purpose is the path planning and collsion avoidance in multi-robot settings by making advantage of communicating and knowing the planned paths of the other robots.

This README helps you with the **setup and usage** of the code. If you are looking for information on the algorithm itself (research, implementation, benchmark performance etc.), please refer to the respective [wiki page](https://github.com/ovgu-FINken/multi_robot_path_planning/wiki/Implemented-Algorithms:-Collvoid).

## Table of content
- [Path costmap (by layer injection)](#path-costmap-by-layer-injection)
  - [Table of content](#table-of-content)
  - [Getting Started](#getting-started)
    - [Requirements](#requirements)
    - [Installation](#installation)
    - [Launching and Testing](#launching-and-testing)
  - [Execution with Benchmark](#execution-with-benchmark)
  - [Common Errors and Unwanted Behavior](#common-errors-and-unwanted-behavior)
  - [Authors](#authors)
  - [License](#license)
  - [Acknowledgments](#acknowledgments)
  
## Getting Started  

Note: Because it is necessary for the integration of this conception to have a movebase, the collvoid-melodic packages are used. Those are unchanged except the costmap_common_params.yaml where the new layer is added.

### Requirements
- Ubuntu 18.04
- ROS melodic
- Gazebo
- C++ 11

- Python 2.7 *
- [PyYAML](https://pypi.org/project/PyYAML/): with `pip install PyYAML` (alternatively, see [here](https://pyyaml.org/wiki/PyYAML)) *

* - needed for collvoid

### Installation

 1. Clone the DrivingSwarm repository: 
      ```
      ~$ git clone https://github.com/ovgu-FINken/DrivingSwarm.git
      ```

  2. Navigate into the workspace: 
      ```
      ~$ cd DrivingSwarm/src
      ```

  3. Checkout the pathplanning branch:
      ```
      ~/DrivingSwarm/src$ git checkout origin/pathplanning 
      ```

  4. Execute:
      ```
      ~/DrivingSwarm/src$ gitman update
      ```

  5. Navigate into the pathplanning repo: 
      ```
      ~/DrivingSwarm/src$ cd pathplanning
      ```

  6. (Checkout the layer_injection branch): 
      ```
       ~/DrivingSwarm/src/pathplanning$ git checkout origin/layer_injection
      ```

  7. Build your workspace with [Catkin Tools: ](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)
      ```
      ~/DrivingSwarm/src/pathplanning$ catkin build
      ```

### Launching and Testing

To start with the extended costmap you just need to normally start an algorithm that uses a move_base for its pathplanning. Make sure that the costmap_common_params.yaml contains the layer as a plugin ( arbitrary name, type: "navigation_path_layers::NavigationPathLayer") and the parameters below it. Also enabled must be set to "true". 

In the following the procedure with the collvoid-dwa-planner from the collvoid-packages is shown.

1. Start the roscore
   ```
   ~$ roscore
   ```

2. Start the standard collvoid script like shown below. **"dwa"** gets started as default planner, otherwise you can also set it specifically by using the **"-p" flag**. The number of robots is set to 3 by default. You can specify the number by using the flag "-n".

    ```
    ~/DrivingSwarm/src/pathplanning$ roscd collvoid_turtlebot/scripts/
    ~/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_turtlebot/scripts$ ./collvoid_std.sh -n 5 -p dwa                                                                                                                                  
    ```

3. After the collvoid_std.sh has finished its setup successfully, you can use the simple goal interface in **RVIZ for testing** it.
   Therefore,

   a. Add the **Tool Properties** to the panels (Menu -> Panels -> Tool Properties)

   b. In Tool Properties, change the Topic name of the 2D Nav Goal to **/tb3_x/move_base_simple/goal**
   where x is the id of the robot.

   c. Place the **2D Nav Goal arrow** at a point and direction of your choice.
   *(Attention: Do not place it on an obstacle or any other area marked black!)*

    ![Image](/collvoid-melodic/res/ScreenshotRVIZ_edited.png)

4.  For a successful test, the robot should move to the defined goal.

5.  You can stop all the processes with:
    ```
    ~/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_turtlebot/scripts$ ./kill.sh`
    ```

## Execution with Benchmark

1. Start the **roscore**
   ```
   ~$ roscore
   ```

2. Set the number of robots in the **settings** file of the benchmark (`benchmark/settings/settings.json`, e.g. 5)

3. Start the **benchmark** script
    ```
    ~/DrivingSwarm/src/pathplanning/benchmark/scripts$ ./benchmark.sh
    ```

4. **Wait** until the spawner has finished (robots should appear in Gazebo), otherwise kill the process with `./kill.sh` and return to 2.

5. Start the **collvoid_benchmark** script. Select the local planner that should be tested (*dwa* in this case) by using the "-p" flag. 
   Set the flag "-n" with the correct number of robots. Example shown below:
    *Note: The number of robots has to be **equivalent** to the number defined in the benchmark settings.* 
    
    ```
    ~/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_turtlebot/scripts$ ./collvoid_benchmark.sh -n 5 -p collvoid
    ```
    
6. Additionally you can start rqt for monitoring at any point.
      ```
      ~/DrivingSwarm/src/pathplanning$ rqt
      ```
      
*Note: Some useful properties to change are located in the settings folder of the benchmark package. First of all, as mentioned above, the "number_of_robots can be set (keep in mind: this value has to be equal to the number of robots) in the collvoid_benchmark.sh command. The world can be changed there, too (maze.world, square.world, tworooms.world) - Caution: in case of "tworooms.world" you have to execute the collvoid_benchmark_tworooms.sh shell script. A third useful setting it the "formation" parameter for the spawning formation. This can either be "dense_block" or "at_way_point". For further information see the [README.md](https://github.com/ovgu-FINken/multi_robot_path_planning/blob/layer_injection/benchmark/README.md) of the benchmark package.*

## Common Errors and Unwanted Behavior

- "Goal Controller"-Node is NoneType and crashes, when trying to send a goal. All processes die and roslaunch will exit. Start again, let about 8-10 seconds between spawning of robots and starting the collvoid-script.
  
- Error with currently unknown cause:  [ WARN] Control loop missed its desired rate of 9.0000Hz... the loop actually took x seconds (usually 100+ seconds)
  
## Authors
Kilian Pößel


## License
This project is licensed under:
- BSD license 

## Acknowledgments
- Johann Schmidt (benchmark-package)
- Nele Traichel (collvoid-melodic-package)
