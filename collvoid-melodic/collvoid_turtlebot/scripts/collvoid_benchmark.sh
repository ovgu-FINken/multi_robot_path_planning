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

## set the number of robots
helpFunction() {
  echo ""
  echo "Please specify the parameter like shown below:"
  echo "$0 -n <same number of robots like in benchmark settings>"
  echo -e "\t e.g. -n 4"
  echo ""
  exit 1 # Exit script after printing help
}

while getopts "n:" opt; do
  case "$opt" in
  n) NUM_ROBOT="$OPTARG" ;;
  ?) helpFunction ;; # Print helpFunction in case parameter is non-existent
  esac
done

# Print helpFunction in case parameters are empty
if [ -z "$NUM_ROBOT" ]; then
  echo ""
  echo "Warning: No parameter value specified."
  helpFunction
  NUM_ROBOT=$NUM_ROBOT_DEFAULT
else
  echo "Collvoid starts with ${NUM_ROBOT} robots."
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
while [ $X -lt $NUM_ROBOT ]; do
  # localisation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "amcl_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot amcl_simple.launch robot:=tb3_${X}" C-m

  # navigation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "move_base_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot move_base.launch robot_name:=tb3_${X}" C-m
  read -t 2

  X=$((X + 1))
done

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