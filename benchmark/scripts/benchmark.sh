#!/usr/bin/env bash

#################################################
# @author:  PathPlanners
# @date:    2019
# @todo:
#################################################

SESSION_NAME="benchmark"
MAPPING=${MAPPING:=amcl}
NUM=0
TERMINAL="gnome-terminal"

# source
source ~/.bashrc

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark world.launch world:=turtlebot3_world.world" C-m
read -t 3

# spawner
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "spawner"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark spawner.launch" C-m

# rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m

# rqt
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rqt"
tmux send-keys -t $SESSION_NAME:$NUM "rqt" C-m

# attach to the tmux session
tmux attach
