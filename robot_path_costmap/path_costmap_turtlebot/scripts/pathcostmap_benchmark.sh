#!/usr/bin/env bash

##################################################################################################
# @author:  Kilian Poessel
# @date:    2020/05/06
# @brief:   Executes the multi-robot navigation with additional costmap layer for robots' paths.
# @todo:
##################################################################################################

# source
#source ~/.bashrc

#### script parameters
SESSION_NAME="pathcostmap"
NUM=0
MAPPING=${MAPPING:=amcl}
WORLD=""
NUM_ROBOT_DEFAULT=3

## set the number of robots
helpFunction() {
  echo ""
  echo "Please specify the parameters like shown below:"
  echo "$0 -n <number_of_robots_like_in_benchmark_settings>"
  echo -e "\t -n 2, or -n 3, ..."
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
  echo "Error: At least one parameter value is not specified."
  helpFunction
  NUM_ROBOT=$NUM_ROBOT_DEFAULT
fi



# start tmux
tmux new-session -s $SESSION_NAME -d

#### simulation incl. spawn is done by benchmark

##### map
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "map_server"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch path_costmap_turtlebot map_server.launch" C-m


#####
# benchmark starts counting from 0 for the robot's index
X=0
while [ $X -lt $NUM_ROBOT ]; do
  # localisation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "amcl_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch path_costmap_turtlebot amcl_simple.launch robot:=tb3_${X}" C-m

  # navigation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "move_base_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch path_costmap_turtlebot move_base.launch robot_name:=tb3_${X}" C-m
  read -t 2
  X=$((X + 1))
done



#### rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m

# attach to the tmux session
tmux attach