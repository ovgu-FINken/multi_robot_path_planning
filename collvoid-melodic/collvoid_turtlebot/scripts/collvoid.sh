#!/usr/bin/env bash

#################################################
# @author:  Nele Traichel
# @date:    2020/01/30
# @brief:   Executes the collvoid multi-robot navigation.
# @todo: * how to work with params??? 
#	 * how to send goal?
#################################################


# source
source ~/.bashrc

# script parameters
SESSION_NAME="collvoid"
NUM=0
MAPPING=${MAPPING:=amcl}
WORLD=""

# start tmux
tmux new-session -s $SESSION_NAME -d

## params ????
#if [ $USE_SETTINGS_FILE == True ] ; then
#  tmux rename-window -t $SESSION_NAME "params"
#  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot set_params.launch" C-m
#fi

# world & spawn
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "world_and_spawn"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot simulation_simple.launch" C-m
fi

##### START AMCL
#### START NODES FOR ROBOTS ---> see collvoid_stage_depr.launch-file

# goal ???
#NUM=$((++NUM))
#tmux new-window -t $SESSION_NAME -n "waypoints"
#tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark waypoints.launch use_settings_file:=$USE_SETTINGS_FILE" C-m


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
