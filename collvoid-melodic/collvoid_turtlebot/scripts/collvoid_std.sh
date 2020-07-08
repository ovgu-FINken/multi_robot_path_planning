#!/usr/bin/env bash

#################################################
# @author:  Nele Traichel
# @date:    2020/01/30
# @brief:   Executes the collvoid multi-robot navigation.
# @todo:
#################################################

# source
#source ~/.bashrc

#### script parameters
SESSION_NAME="collvoid"
NUM=0
MAPPING=${MAPPING:=amcl}
WORLD=""
NUM_ROBOT_DEFAULT=3
PLANNER_DEFAULT="dwa"
COLLVOID="collvoid"
DWA="dwa"

## set the number of robots
helpFunction() {
  echo "In case you want to adjust the values, the following parameters are available:"
  echo "$0 -p <planner> -n <number of robots like in benchmark settings>"
  echo -e "\t -p collvoid, or -p dwa"
  echo -e "\t -n 4"
  echo ""
}

while getopts "n:p:" opt; do
  case "$opt" in
  p) PLANNER="$OPTARG" ;;
  n) NUM_ROBOT="$OPTARG" ;;
  ?) helpFunction ;; # Print helpFunction in case parameter is non-existent
  esac
done

# Print helpFunction in case parameters are empty
if [ -z "$PLANNER" ]; then
  echo ""
  echo "Warning: No local planner specified. Will use ${PLANNER_DEFAULT}_local_planner by default."
  helpFunction
  PLANNER=$PLANNER_DEFAULT
fi

if [ -z "$NUM_ROBOT" ]; then
  NUM_ROBOT=$NUM_ROBOT_DEFAULT
else
  echo "Be aware that this won't have an effect on the simulation, except you have changed the simulation launch file to a scalable one."
fi

if [ "$PLANNER" = "$COLLVOID" ] || [ "$PLANNER" = "$DWA" ]; then
  echo ""
  echo "Move base is starting with ${PLANNER}_local_planner and ${NUM_ROBOT} robots ..."
  echo ""
else
  echo ""
  echo "There is no such planner. Please, check the spelling."
  echo ""
  exit 1 # Exit script after printing help
fi

# start tmux
tmux new-session -s $SESSION_NAME -d

#### simulation incl. spawn
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "world_and_spawn"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot simulation_simple.launch" C-m
read -t 3

##### map
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "map_server"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot map_server.launch" C-m

#####
X=0
if [[ "$PLANNER" == "$COLLVOID" ]]; then
  while [ $X -lt $NUM_ROBOT ]; do
    X=$((X + 1))
    # localisation
    NUM=$((++NUM))
    tmux new-window -t $SESSION_NAME -n "amcl_${X}"
    tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot amcl_simple.launch robot:=tb3_${X}" C-m

    # navigation
    NUM=$((++NUM))
    tmux new-window -t $SESSION_NAME -n "move_base_${X}"
    tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot move_base_collvoid.launch robot_name:=tb3_${X}" C-m
    read -t 3
  done
elif [ "$PLANNER" = "$DWA" ]; then
  while [ $X -lt $NUM_ROBOT ]; do
    X=$((X + 1))
    # localisation
    NUM=$((++NUM))
    tmux new-window -t $SESSION_NAME -n "amcl_${X}"
    tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot amcl_simple.launch robot:=tb3_${X}" C-m

    # navigation
    NUM=$((++NUM))
    tmux new-window -t $SESSION_NAME -n "move_base_${X}"
    tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot move_base_dwa.launch robot_name:=tb3_${X}" C-m
    read -t 3
  done
fi

#### rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m
read -t 3

# attach to the tmux session
tmux attach
