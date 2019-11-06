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

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_world.launch" C-m

# random walk
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "random"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_simulation.launch" C-m

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
