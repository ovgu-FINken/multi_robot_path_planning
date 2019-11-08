#!/usr/bin/env bash

#################################################
# @author:  PathPlanners
# @date:    2019
# @todo:
#################################################

SESSION_NAME="multi_robo"
NUM=0

# source
source ~/.bashrc
source ~/git/DrivingSwarm/devel/setup.bash
source ~/git/DrivingSwarm//setup.bash
# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot simulation.launch" C-m

# amcl
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "amcl"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch tracking_uncertainty_grid amcl.launch" C-m

# main
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "main"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot main.launch" C-m

# rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m

# rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rqt"
tmux send-keys -t $SESSION_NAME:$NUM "rqt" C-m

# attach to the tmux session
tmux attach
