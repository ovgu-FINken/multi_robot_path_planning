#!/usr/bin/env bash

#################################################
# @author:  Viviane Wolters
# @date:    04/2020
# @todo:
#################################################

PKG_NAME="cocktailparty_algorithm"
SESSION_NAME="Simulation_Bug2"
NUM=0
MAPPING=${MAPPING:=amcl}
WORLD=""
NUM_ROBOT=1

source ~/.bashrc

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch cocktailparty_algorithm simulation_simple.launch" C-m

X=0
while [ $X -lt $NUM_ROBOT ]; do
  # localisation
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "amcl_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch cocktailparty_algorithm amcl.launch robot_name:=tb3_${X}" C-m

  X=$((X + 1))
done

# attach to the tmux session
tmux attach
