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

## Note: there is no flag here because the number of robots cannot be increased in the simulation with the launch file this script is using.
NUM_ROBOT=$NUM_ROBOT_DEFAULT
echo "Collvoid starts with ${NUM_ROBOT} robots."

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
while [ $X -lt $NUM_ROBOT ]; do
  X=$((X + 1))
  # localisation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "amcl_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot amcl_simple.launch robot:=tb3_${X}" C-m

  # navigation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "move_base_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot move_base.launch robot_name:=tb3_${X}" C-m
  read -t 3

done

#### rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m
read -t 3

# attach to the tmux session
tmux attach
