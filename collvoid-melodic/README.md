# Collvoid Algorithm (melodic dist)
Welcome to multi-robot path planning with the Collvoid algorithm! 

The code is an adapted migration of [this ROS package](http://wiki.ros.org/multi_robot_collision_avoidance) (to melodic distribution), based on the algorithm described in [this paper](http://citeseerx.ist.psu.edu/viewdoc/download;jsessionid=4E0E826AEF1141BCD1F23906F0116645?doi=10.1.1.386.6439&rep=rep1&type=pdf). Its purpose is the path planning and **coll**ision a**void**ance in multi-robot settings by making advantage of velocity obstacles.

This README helps you with the ***setup and usage*** of the code. If you are looking for information on the algorithm itself (research, implementation, performance etc.), please refer to the respective [wiki page](https://github.com/ovgu-FINken/multi_robot_path_planning/wiki/Implemented-Algorithms:-Collvoid-Algorithm-melodic-dist).


<!-- TOC START min:1 max:5 link:true asterisk:false update:true -->
## Table of content
- [Getting Started](#Getting-Started)
  - [Requirements](#Requirements)
  - [Installation](#Installation)
  - [Built With](#Built-With)
  - [Launching and Testing](#Launching-and-Testing)
- [Common Errors](#Common-Errors)
- [Execution with Benchmark]()
- [Authors](#Authors)
- [License](#License)
- [Acknowledgments](#Acknowledgments)

<!-- TOC END -->
## Getting Started
### Requirements
*What things you need to install the software and how to install them*

- Ubuntu 18.04
- ROS melodic
- Gazebo
- Python 2.7
- C++ 11
- [PyYAML](https://pypi.org/project/PyYAML/): with `pip install PyYAML` (alternatively, see [here](https://pyyaml.org/wiki/PyYAML))

### Installation
*A step by step series of examples that tell you how to get a development env running. End with an example of getting some data out of the system.*
1. clone the repository
2. TODO


### Built with 
[Catkin Tools: ](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)
`catkin build`

### Launching and Testing
Start the standard script like in the example below. For setting the number of robots, please, make use of the respective flag "-n" (default value = 4).
```
~/DrivingSwarm/src/pathplanning$ cd collvoid-melodic/collvoid_turtlebot/scripts/
~/DrivingSwarm/src/pathplanning$ ./collvoid_std.sh -n 3
```

## Common Errors
TODO

## Execution with Benchmark
1. Start the benchmark script (benchmark.sh)
2. Wait until the spawner has finished (robots should appear in Gazebo)
3. Start the collvoid_benchmark script (see example below)


*Note*, that the number of robots has to be **equivalent** to the number defined in the benchmark (benchmark/settings/settings.json). Therefore, make use of the respective flag "-n" like in the example below (default value = 4). 
```
~/DrivingSwarm/src/pathplanning$ cd collvoid-melodic/collvoid_turtlebot/scripts/
~/DrivingSwarm/src/pathplanning$ ./collvoid_benchmark.sh -n 5
```

## Authors
Nele Traichel


## License
Due to the usage of external code and libraries this project is (partially?) licensed under:
- BSD license (multi_robot_collision_avoidance)
- proprietary license ([RVO2 library](http://gamma.cs.unc.edu/RVO2/documentation/2.0/))

## Acknowledgments
- Daniel Claes (code base in ROS indigo distro: http://wiki.ros.org/multi_robot_collision_avoidance)
- Johann Schmidt (goal_controller based on his movement_controller in benchmark-pkg)
