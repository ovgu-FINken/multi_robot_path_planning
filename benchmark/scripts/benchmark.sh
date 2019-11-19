#!/usr/bin/env bash

#################################################
# @author:  PathPlanners
# @date:    2019
# @brief:
# @todo:
#################################################
#
# USAGE RECOMMENDATION
#
# -> Create shortcuts like:
# -> gnome-terminal -- bash <FULL_PATH>/benchmark.sh
# -> gnome-terminal -- bash <FULL_PATH>/kill.sh
#
#################################################


ENABLE_RVIZ=False
ENABLE_RQT=True

SESSION_NAME="benchmark"
MAPPING=${MAPPING:=amcl}
NUM=0

# source
source ~/.bashrc

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark world.launch world:=turtlebot3_world.world" C-m
read -t 3

# mapping @HACK
ROBOT_NAMES=(0 1 2 3)
for i in "${ROBOT_NAMES[@]}"; do
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "mapping_tb3_${i}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark amcl.launch robot_name:=tb3_${i}" C-m
done

# spawner
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "spawner"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark spawner.launch" C-m
read -t 3

# waypoints
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "waypoints"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark waypoints.launch" C-m

# movement
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "movement"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark movement.launch" C-m


# rviz
if [ $ENABLE_RVIZ == True ] ; then
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "rviz"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m
fi

# rqt
if [ $ENABLE_RQT == True ] ; then
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "rqt"
  tmux send-keys -t $SESSION_NAME:$NUM "rqt" C-m
fi

# attach to the tmux session
tmux attach
