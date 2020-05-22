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
  echo ""
  echo "Please specify the parameters like shown below:"
  echo "$0 -p <local_planner> -n <number_of_robots_like_in_benchmark_settings>"
  echo -e "\t -p collvoid, or -p dwa"
  echo -e "\t -n 2, or -n 3, ..."
  echo ""
  exit 1 # Exit script after printing help
}

while getopts "n:p:" opt; do
  case "$opt" in
  p) PLANNER="$OPTARG" ;;
  n) NUM_ROBOT="$OPTARG" ;;
  ?) helpFunction ;; # Print helpFunction in case parameter is non-existent
  esac
done

# Print helpFunction in case parameters are empty
if [ -z "$PLANNER" ] || [ -z "$NUM_ROBOT" ]; then
  echo ""
  echo "Error: At least one parameter value is not specified."
  helpFunction
  NUM_ROBOT=$NUM_ROBOT_DEFAULT
fi

if [ "$PLANNER" = "$COLLVOID" ] || [ "$PLANNER" = "$DWA" ]; then
  echo ""
  echo "Move base is starting with ${PLANNER}_local_planner and ${NUM_ROBOT} robots ..."
  echo ""
else
  echo ""
  echo "There is no such planner. Please, check the spelling."
  helpFunction
fi


# start tmux
tmux new-session -s $SESSION_NAME -d

#### simulation incl. spawn is done by benchmark

##### map
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "map_server"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot map_server.launch" C-m

#####
# benchmark starts counting from 0 for the robot's index
X=0
if [[ "$PLANNER" == "$COLLVOID" ]]; then
  while [ $X -lt $NUM_ROBOT ]; do
    # localisation
    NUM=$((++NUM))
    tmux new-window -t $SESSION_NAME -n "amcl_${X}"
    tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot amcl_simple.launch robot:=tb3_${X}" C-m

    # navigation
    NUM=$((++NUM))
    tmux new-window -t $SESSION_NAME -n "move_base_${X}"
    tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot move_base_collvoid.launch robot_name:=tb3_${X}" C-m
    read -t 2
    X=$((X + 1))
  done
elif [ "$PLANNER" = "$DWA" ]; then
  while [ $X -lt $NUM_ROBOT ]; do
    # localisation
    NUM=$((++NUM))
    tmux new-window -t $SESSION_NAME -n "amcl_${X}"
    tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot amcl_simple.launch robot:=tb3_${X}" C-m

    # navigation
    NUM=$((++NUM))
    tmux new-window -t $SESSION_NAME -n "move_base_${X}"
    tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot move_base_dwa.launch robot_name:=tb3_${X}" C-m
    read -t 2
    X=$((X + 1))
  done
fi

# goal controller
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "goal_controller"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot goal_controller.launch" C-m

#### rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m

# attach to the tmux session
tmux attach
