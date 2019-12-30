# Path Planning Benchmark
Welcome to the Path Planning Benchmark!
1. [System](#system)
2. [Settings](#settings)
3. [Worlds](#worlds)
4. [Execution](#execution)
5. [Evaluation](#evaluation)

## System
This framework was successfully tested under:
- Ubuntu 18.04
- ROS Melodic
- Python 2.7

## Settings

All adjustable parameter can be found in the settings file: `settings/settings.json`:
```
{
  "model_name": "turtlebot3",
  "model_type": "burger",
  "namespace": "tb3_",
  "number_of_robots": 4,
  "formation": "dense_block",
  "position": [1.5, 0.5, 0.5],
  "orientation": [0.0, 0.0, 0.0],
  "wp_map": "tb3_edge",
  "wp_threshold": 0.2,
  "world": "turtlebot3.world",
  "rounds": 1,
  "end_procedure": "despawn"
}
```

#### World Name
The **Model Name** defines the robot model to be used.
This can be set to: `turltebot3`.

#### Model Type
The **Model Type** defines the type of the robot model.
This can be set to: `burger`, `waffle`, or `waffle_pi`
for the `turtlebot3` model.

#### Namespace
The **Namespace** defines the String, which will be put in front
of the robot name for all topics and so on.
This can be set to every arbitrary String.

#### Number of Robots
The **Number of Robots** defines the number of robots, which should
be spawned and used in the benchmark.
This can be set to every Integer.

#### Formation
The **Formation** defines where the robots should be spawned.
This can be set to `dense_block` or `at_way_point`.

#### Position
The **Position** defines the position in the map for the spawning.
This is only be used, when the formation is set to `dense_block`, since
for other formations the positions are automatically computed.
This can be set to any valid position vector `[x, y, z]` in the map.

#### Orientation
The **Orientation** defines the orientation of the formation. This is 
similar to the position parameter and will only be used in the `dense_block`
formation. This can be set to any valid orientation euler vector.

#### Waypoint Map
The **Waypoint Map** defines the set of waypoints to be used.
This can be set to `maze` or `tb3_edge`. CAUTION: These waypoint maps are 
tailored to specific world files. Please, read the [world docs](doc).

#### Waypoint Threshold
The **Waypoint Threshold** defines the size of the waypoint.
More specifically, it defines the radius of the waypoint.
This can be set to any Float.

#### World Map
The **World** defines the gazebo world file, which will be loaded by 
gazebo after the execution of the benchmark.
This can be set to `maze.world`, `square.world`, `turtlebot3.world`, or
`tworooms.world` (All world files are located in the [world folder](worlds).).

#### Rounds
The **Rounds** variable defines the number the benchmark should run. 
Each waypoint map as a finite set of waypoints, once the initial waypoint 
is reached again one round is fulfilled. When all rounds are accomplished,
the benchmark finished.
This can be set to any Integer.

#### End Procedure
The **End Procedure** defines how the robots will behave after they 
successfully finished all rounds. Here, this can be set to `despawn`, which 
simply deletes the robot model. This, however, is only applicable in
simulation-based benchmarks. Alternatively, this variable can be set to 
`stay`, which will freeze the robot at the last waypoint, whereas the value 
`start` will send the start position as the next goal, such that 
the robot will move to the start position after the last waypoint is reached.
Ultimately, the `ìdle` state is a free state, where the robots might just move 
randomly, which is up to the user.

#### RQT & RVIZ
Furthermore, RQT and RVIZ can be started by enabling
the `ENABLE_RQT` and `ENABLE_RVIZ` flags, respectively.
These flags can be found in the `scripts/benchmark.sh` file.

## Worlds
The following world files are supported:
* [TB3 World](doc/TB3_WORLD.md)
* [Maze World](doc/MAZE_WORLD.md)
* [Warehouse World](doc/WAREHOUSE_WORLD.md)

Please, refer to these document files for more information, like how to 
use these world maps and the corresponding waypoint maps.

## Execution

First, make sure you are in the benchmark package.
The benchmark can be executed via:
```
bash ./scripts/benchmark.sh
```
The framework can be closed by typing:
```
bash ./scripts/kill.sh
```
This process can be highly accelerated by creating system keyboard shortcuts like:
```
gnome-terminal -- bash <FULL_PATH>/benchmark.sh
gnome-terminal -- bash <FULL_PATH>/kill.sh
```

## Evaluation

Whenever a robot is reaching a new waypoint the duration to reach this goal
is tracked and printed into the console, as well as stored in an [eval log](log/eval_log.txt).

Furthermore, for purely simulation-based benchmarks, the following benchmark scenario can be used 
to evaluate the performance for the path planner:
"Each benchmark file has a list of start/goal locations.
The intention is that one would add one agent at a time until
an algorithm cannot solve a problem in a given time/memory limit."
([Reference](https://movingai.com/benchmarks/mapf.html))

### Clean log files
All log files can be cleaned (content deletion) via the following shell script:
```
bash scripts/clean_log.sh
```