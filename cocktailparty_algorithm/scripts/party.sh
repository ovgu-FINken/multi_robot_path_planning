#!/usr/bin/env bash

#################################################
# @author:  Viviane Wolters
# @date:    04/2020
# @todo:
#################################################

SESSION_NAME="cocktailparty_algorithm"
PKG_NAME="cocktailparty_algorithm"
NUM=0

# source
source ~/.bashrc
source ~/git/DrivingSwarm/devel/setup.bash
source ~/git/DrivingSwarm//setup.bash

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch ${PGK_NAME} simulation_simple.launch" C-m

# amcl
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "amcl"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch ${PGK_NAME} amcl_simple.launch" C-m

# attach to the tmux session
tmux attach
