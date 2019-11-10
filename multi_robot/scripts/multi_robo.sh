#!/usr/bin/env bash

#################################################
# @author:  PathPlanners
# @date:    2019
# @todo:
#################################################

SESSION_NAME="multi_robo"
MAPPING=${MAPPING:=amcl}
NUM=0
TERMINAL="gnome-terminal"

# source
source ~/.bashrc
#source ~/git/DrivingSwarm/devel/local_setup.bash

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot simulation.launch world:=turtlebot3_world.world" C-m
read -t 3

# navigation
#NUM=$((++NUM))
#tmux new-window -t turtlebots -n "navigation"
#tmux send-keys -t turtlebots:$NUM "roslaunch multi_robot tracking_simulation.launch" C-m

# main
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "main"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot main.launch" C-m


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
