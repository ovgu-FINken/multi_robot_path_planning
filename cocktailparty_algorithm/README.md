# Cocktail-Party-Algorithms
Welcome to multi-robot path planning with the Cocktail-Party-Algorithms! 

Don't be confused by the name of the project, it's just a working title.

Imagine a cocktail party. A guest decides to talk to someone. He accomplishes this by maneuvering between tables,
chairs, and other guests, planning his path “on the fly” and not consulting with other people about his or their intended motion.
He assumes that other people mean well, and so as long as he somehow takes into account their movements, 
it is safe to move at a minimal distance from them. 

The goal of this project was to implement exactly this behaviour on multiple robots. It was archieved by implementing a simple, for this purpose adapted, **Bug2** algorithm which was later transformed to a **TangentBug** algorithm.

This README helps you with the ***setup and usage*** of the code. If you are looking for information on the algorithms itself (research, implementation, benchmark performance etc.), please refer to the respective [wiki page](https://github.com/ovgu-FINken/multi_robot_path_planning/wiki/Implemented-Algorithms:-Collvoid).


<!-- TOC START min:1 max:5 link:true asterisk:false update:true -->
## Table of content
- [Cocktail-Party-Algorithms](#cocktail-party-algorithms)
  - [Table of content](#table-of-content)
  - [Getting Started](#getting-started)
    - [Requirements](#requirements)
    - [Installation](#installation)
    - [Launching and Testing](#launching-and-testing)
  - [Execution with Benchmark](#execution-with-benchmark)
  - [Common Errors and Unwanted Behavior](#common-errors-and-unwanted-behavior)
  - [Author](#author)


<!-- TOC END -->
## Getting Started
### Requirements
- Ubuntu 18.04
- ROS melodic
- Gazebo
- Python 2.7

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
      ~/DrivingSwarm/src$ cd pathplanning      ```

  6. (Checkout the cocktailparty_algorithm branch): 
      ```
       ~/DrivingSwarm/src/pathplanning$ git checkout origin/cocktailparty_algorithm
      ```
  7. Build your workspace with [Catkin Tools: ](https://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html)
      ```
      ~/DrivingSwarm/src/pathplanning$ catkin build
      ```

### Launching and Testing 
Both algorithms (Bug2 and TangentBug) can only be tested by using the benchmark.
For a successful execution you need the lightweight and flexible command-line JSON processor jq. You can install it with:
```
~$ sudo apt install jq
```
1. Start the **roscore**
   ```
   ~$ roscore
   ```   
2. Set the parameters in the **settings** file of the benchmark (`benchmark/settings/settings.json`)
   For detail information: [README Benchmark](https://github.com/ovgu-FINken/multi_robot_path_planning/blob/benchmark/benchmark/README.md)

3. Start the **benchmark** script
   ```
   ~/DrivingSwarm/src/pathplanning/cocktailparty_algorithm/scripts$ ./benchmark.sh
   ```
4. **Wait** until the spawner has finished (robots should appear in Gazebo), otherwise kill the process with `./kill.sh` and return to 3.

5. Start the **Bug2** or the **TangentBug** script.
   ```
   ~/DrivingSwarm/src/pathplanning/cocktailparty_algorithm/scripts$ ./bug2.sh
   ```
   **or**
   ```
   ~/DrivingSwarm/src/pathplanning/cocktailparty_algorithm/scripts$ ./tangentBug.sh
   ```
6. For a successful test, the robots should move to the defined goals and finish the benchmark.

7. You can stop all the processes with:
   ```
   ~/DrivingSwarm/src/pathplanning/cocktailparty_algorithm/scripts$ ./kill.sh`
   ```
                                                           
## Author
Viviane Wolters
