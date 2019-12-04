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
USE_SETTINGS_FILE=True

# start tmux
tmux new-session -s $SESSION_NAME -d

# settings
tmux rename-window -t $SESSION_NAME "settings"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark settings.launch" C-m

# world
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark world.launch world:=turtlebot3_world.world" C-m

# spawner
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "spawner"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark spawner.launch use_settings_file=$USE_SETTINGS_FILE" C-m

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
