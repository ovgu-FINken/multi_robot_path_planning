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



## Execution with Benchmark

x. Additionally you can start rqt for monitoring
      ```
      ~/DrivingSwarm/src/pathplanning$ rqt
      ```

## Common Errors and Unwanted Behavior
  
## Authors
Kilian Pößel


## License
This project is licensed under:
- BSD license 

## Acknowledgments
- Johann Schmidt (benchmark-package)
- Nele Traichel (collvoid-melodic-package)
