#!/usr/bin/env bash

#################################################
# @author:  Johann Schmidt
# @date:    2019/20
# @brief:   Executes the benchmark.
# @todo:
#################################################

# source
# shellcheck source=src/lib.sh
source ~/.bashrc

# script parameters
SESSION_NAME="benchmark"
NUM=0
ENABLE_RVIZ=False
ENABLE_RQT=False
USE_SETTINGS_FILE=True
DEFAULT_MOVEMENT=False

# start tmux
tmux new-session -s $SESSION_NAME -d

# settings
if [ $USE_SETTINGS_FILE == True ]; then
  tmux rename-window -t $SESSION_NAME "settings"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark settings.launch" C-m
fi
read -t 1

# world
if [ $USE_SETTINGS_FILE == True ]; then
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "world"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark world.launch use_settings_file:=$USE_SETTINGS_FILE" C-m
else
  tmux rename-window -t $SESSION_NAME "world"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark world.launch use_settings_file:=$USE_SETTINGS_FILE" C-m
fi

# spawner
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "spawner"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark spawner.launch use_settings_file:=$USE_SETTINGS_FILE" C-m

# waypoints
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "waypoints"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark waypoints.launch use_settings_file:=$USE_SETTINGS_FILE" C-m

# movement
if [ $DEFAULT_MOVEMENT == True ]; then
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "movement"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark movement.launch" C-m
fi

# evaluation
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "evaluation"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark evaluation.launch use_settings_file:=$USE_SETTINGS_FILE" C-m


# rviz
if [ $ENABLE_RVIZ == True ]; then
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "rviz"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m
fi

# rqt
if [ $ENABLE_RQT == True ]; then
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "rqt"
  tmux send-keys -t $SESSION_NAME:$NUM "rqt" C-m
fi

# attach to the tmux session
tmux attach
