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
SESSION_NAME="test"
NUM=0
# set the number of robots
NUM_ROBOT=4
NUM_ROBOT=$1

# start tmux
tmux new-session -s $SESSION_NAME -d

# start waypoint publisher that operates as a test
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "test"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch collvoid_turtlebot test.launch" C-m

# attach to the tmux session
tmux attach
