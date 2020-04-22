# Collvoid Algorithm (melodic dist)
Welcome to multi-robot path planning with the Collvoid algorithm! 

The code is an adapted migration of [this ROS package](http://wiki.ros.org/multi_robot_collision_avoidance) (to melodic distribution), based on the algorithm described in [this paper](http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=4E0E826AEF1141BCD1F23906F0116645?doi=10.1.1.386.6439&rep=rep1&type=pdf). Its purpose is the path planning and **coll**ision a**void**ance in multi-robot settings by making advantage of velocity obstacles.

This README helps you with the ***setup and usage*** of the code. If you are looking for information on the algorithm itself (research, implementation, performance etc.), please refer to the respective [wiki page](https://github.com/ovgu-FINken/multi_robot_path_planning/wiki/Implemented-Algorithms:-Collvoid-Algorithm-melodic-dist).


<!-- TOC START min:1 max:5 link:true asterisk:false update:true -->
## Table of content
- [Collvoid Algorithm (melodic dist)](#collvoid-algorithm-melodic-dist)
  - [Table of content](#table-of-content)
  - [Getting Started](#getting-started)
    - [Requirements](#requirements)
    - [Installation](#installation)
    - [Launching and Testing](#launching-and-testing)
  - [Common Errors](#common-errors)
  - [Execution with Benchmark](#execution-with-benchmark)
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
1. Start the standard collvoid script like shown below. Due to the simulation launcher the **number of robots** is set to 3 by default 

    *Note: In the future, in case the simulation is enhanced such that an arbitrary amount of robots can be spawned, 
      you can speficy the number by using the flag "-n".*

    ```
    ~/DrivingSwarm/src/pathplanning$ cd collvoid-melodic/collvoid_turtlebot/scripts/
    ~/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_turtlebot/scripts$ ./collvoid_std.sh
    ```

2. After the collvoid_std.sh has finished its setup successfully, you can use the simple goal interface in **RVIZ** for testing it.
   Therefore,

   a. Add the **Tool Properties** to the panels (Menu -> Panels -> Tool Properties)

   b. In Tool Properties, change the Topic name of the 2D Nav Goal to **/tb3_x/move_base_simple/goal**

   c. Place the **2D Nav Goal arrow** at a point and direction of your choice.
   *(Attention: Do not place it on an obstacle or any other black area!)*

    [[/res/ScreenshotRVIZ_edited.png]]

3.  For a successful test, all robots should move to the defined goal.

4.  You can stop all the processes with:
    ```
    ~/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_turtlebot/scripts$ ./kill.sh`
    ```

## Common Errors
There are some race conditions that can cause trouble. 
That's why, killing and restarting the script can help in many cases.

## Execution with Benchmark
1. Set the number of robots in the settings file of the benchmark (`benchmark/settings/settings.json`, e.g. 5)

2. Start the benchmark script
    ```
    ~/DrivingSwarm/src/pathplanning/benchmark/scripts$ ./benchmark.sh
    ```

3. Wait until the spawner has finished (robots should appear in Gazebo), otherwise kill the process with `./kill.sh` and return to 2.

4. Start the collvoid_benchmark script. Set the flag "-n" with the correct number of robots (otherwise the number will be set to 3)

    *Note: The number of robots has to be **equivalent** to the number defined in the benchmark settings.* 
    ```
    ~/DrivingSwarm/src/pathplanning/collvoid-melodic/collvoid_turtlebot/scripts$ ./collvoid_benchmark.sh -n 5
    ```


## Authors
Nele Traichel


## License
Due to the usage of external code and libraries this project is (partially?) licensed under:
- BSD license 
- proprietary license ([RVO2 library](http://gamma.cs.unc.edu/RVO2/documentation/2.0/))

## Acknowledgments
- Daniel Claes (code base in ROS indigo distro: http://wiki.ros.org/multi_robot_collision_avoidance)
- Johann Schmidt (goal_controller based on his movement_controller in benchmark-pkg)
