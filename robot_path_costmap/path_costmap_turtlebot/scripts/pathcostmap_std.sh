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
  echo "In case you want to adjust the values, the following parameters are available:"
  echo "$0 -n <number of robots like in benchmark settings>"
  echo -e "\t -n 4"
  echo ""
}

while getopts "n:" opt; do
  case "$opt" in
  n) NUM_ROBOT="$OPTARG" ;;
  ?) helpFunction ;; # Print helpFunction in case parameter is non-existent
  esac
done

# Print helpFunction in case parameters are empty

if [ -z "$NUM_ROBOT" ]; then
  NUM_ROBOT=$NUM_ROBOT_DEFAULT
else
  echo "Be aware that this won't have an effect on the simulation, except you have changed the simulation launch file to a scalable one."
fi

# start tmux
tmux new-session -s $SESSION_NAME -d

#### simulation incl. spawn
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "world_and_spawn"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch path_costmap_turtlebot simulation_simple.launch" C-m
read -t 3

##### map
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "map_server"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch path_costmap_turtlebot map_server.launch" C-m

#####
X=0
while [ $X -lt $NUM_ROBOT ]; do
  X=$((X + 1))
  # localisation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "amcl_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch path_costmap_turtlebot amcl_simple.launch robot:=tb3_${X}" C-m

  # navigation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "move_base_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch path_costmap_turtlebot move_base_dwa.launch robot_name:=tb3_${X}" C-m
  read -t 3
done

#### rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m
read -t 3

# attach to the tmux session
tmux attach