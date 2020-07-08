# Collvoid Algorithm (melodic dist)
Welcome to multi-robot path planning with the Collvoid algorithm! 

The code is an adapted migration of [this ROS package](http://wiki.ros.org/multi_robot_collision_avoidance) (to melodic distribution), based on the algorithm described in [this paper](http://www.willowgarage.com/sites/default/files/2012%20IROS%20Collision%20avoidance%20under%20bounded%20localization%20uncertainty.pdf). 
Its purpose is the path planning and **coll**ision a**void**ance in multi-robot settings by making advantage of velocity obstacles.

This README helps you with the ***setup and usage*** of the code. If you are looking for information on the algorithm itself (research, implementation, benchmark performance etc.), please refer to the respective [wiki page](https://github.com/ovgu-FINken/multi_robot_path_planning/wiki/Implemented-Algorithms:-Collvoid).


<!-- TOC START min:1 max:5 link:true asterisk:false update:true -->
## Table of content
- [Collvoid Algorithm (melodic dist)](#collvoid-algorithm-melodic-dist)
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

<!-- TOC END -->
## Getting Started
### Requirements
- Ubuntu 18.04
- ROS melodic
- Gazebo
- Python 2.7
- C++ 11
- [PyYAML](https://pypi.org/project/PyYAML/): with `pip install PyYAML` (alternatively, see [here](https://pyyaml.org/wiki/PyYAML))

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

  6. (Checkout the collvoid branch): 
      ```
       ~/DrivingSwarm/src/pathplanning$ git checkout origin/collvoid
      ```

  7. Build your workspace with [Catkin Tools: ](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)
      ```
      ~/DrivingSwarm/src/pathplanning$ catkin build
      ```

### Launching and Testing
1. Start the roscore
   ```
   ~$ roscore
   ```

2. Start the **standard collvoid script** like shown below. Define the **local planner** (*dwa*, or *collvoid*) by using the **"-p" flag**, else "dwa" gets started by default.
   The number of robots is set to 3 by default.

    *Note: In the future, in case the simulation is enhanced such that an arbitrary amount of robots can be spawned, 
      you can specify the number by using the flag "-n".*

    ```
    ~/DrivingSwarm/src/pathplanning$ roscd collvoid_turtlebot/scripts/
    ~/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_turtlebot/scripts$ ./collvoid_std.sh -p collvoid                                                                                                                                  
    ```

3. After the collvoid_std.sh has finished its setup successfully, you can use the simple goal interface in **RVIZ for testing** it.
   Therefore,

   a. Add the **Tool Properties** to the panels (Menu -> Panels -> Tool Properties)

   b. In Tool Properties, change the Topic name of the 2D Nav Goal to **/tb3_x/move_base_simple/goal**
   where x is the id of the robot **(0, 1, or 2)**

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

5. Start the **collvoid_benchmark** script. Select the local planner that should be tested (*dwa*, or *collvoid*) by using the "-p" flag. 
   Set the flag "-n" with the correct number of robots. Example shown below:

    *Note: The number of robots has to be **equivalent** to the number defined in the benchmark settings.* 
    ```
    ~/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_turtlebot/scripts$ ./collvoid_benchmark.sh -n 5 -p collvoid
    ```


## Common Errors and Unwanted Behavior

*Note:* Be patient when restarting, all processes should stop completely before executing the restart.

***Non-specific to planner:***

| Error/Warning Message | behavior                            | presumed cause                                                                                  | what to do (short-term)?                         | (considerations to resolve error long-term)             |
| :-------------------- | ----------------------------------- | ----------------------------------------------------------------------------------------------- | ------------------------------------------------ | ------------------------------------------------------- |
| "could not get me"    | robot does not start moving         | race conditions                                                                                 | Killing the process and restarting the script    | identifying point of dependency and adding waiting time |
| -                     | robot arrives at its goal and stops | benchmark controller did not register the goal as reached and thus, does not send the next goal | Lowering the goal tolerance of the local planner | -                                                       |


***DWA Local Planner:***

| Error/Warning Message                                          | behavior           | presumed cause                    | what to do (short-term)?                                                          | (considerations to resolve error long-term)                                                                      |
| :------------------------------------------------------------- | ------------------ | --------------------------------- | --------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- |
| "Extrapolation Error: Lookup would require extrapolation ..."  | ?                  | delays in publishing tf data      | ignore it, unless it leads to unwanted behavior of the robot (in that case, dto.) | identifying the source of delay, there are a lot of forum entries discussing this problem                        |
| Repetition of "Got new plan" and "DWA failed to produce path." | robot stops moving | ? (maybe the extrapolation error) | dto.                                                                              | fixing existing recovery behavior (in dwa_planner_ros) & if necessary, adding another recovery behavior          |
| ?                                                              | oscillation        | ?                                 | dto.                                                                              | fixing the existing oscillation recovery behavior (maybe testing different oscillation parameters, e.g. timeout) |

***Collvoid Local Planner:***

| Error/Warning Message                                          | behavior                                     | presumed cause                                                                                           | what to do (short-term)?     | (considerations to resolve error long-term)          |
| :------------------------------------------------------------- | -------------------------------------------- | -------------------------------------------------------------------------------------------------------- | ---------------------------- | ---------------------------------------------------- |
| "Could not get Obstacles from service"                         | -                                            | in the original code obstacles can be detected using opencv but this is not used here                    | ignore it                    | either delete the whole section or make use of it    |
| "Did not find safe velocity, choosing outside constraints ..." | -                                            | -                                                                                                        | ignore it                    | adapting the constraint handling to turtlebot3       |
| -                                                              | collisions between robots, or robot and wall | -                                                                                                        | ignore it, or restart script | testing different parameters, else editing code ...? |
| -                                                              | robots do not reverse                        | constraints handling is not adaptable to other robot kinematics, or negative velocities are discarded(?) | -                            | identify actual source of unwanted behavior          |

## Authors
Nele Traichel


## License
Due to the usage of external code and libraries this project is (partially?) licensed under:
- BSD license 
- proprietary license ([RVO2 library](http://gamma.cs.unc.edu/RVO2/documentation/2.0/))

## Acknowledgments
- Daniel Claes (code base in ROS indigo distro: http://wiki.ros.org/multi_robot_collision_avoidance)
- Johann Schmidt (goal_controller based on his movement_controller in benchmark-pkg)
