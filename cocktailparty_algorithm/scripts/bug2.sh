#!/usr/bin/env bash

#################################################
# @author:  Viviane Wolters
# @date:    04/2020
# @todo:
#################################################

PKG_NAME="cocktailparty_algorithm"
SESSION_NAME="Bug2"
NUM=0
NUM_ROBOT=5

source ~/.bashrc

# start tmux
tmux new-session -s $SESSION_NAME -d

X=0
while [ $X -lt $NUM_ROBOT ]; do

  # #localisation
  # NUM=$((++NUM))
  # tmux new-window -t $SESSION_NAME -n "amcl_${X}"
  # tmux send-keys -t $SESSION_NAME:$NUM "roslaunch cocktailparty_algorithm amcl.launch robot:=tb3_${X}" C-m


  # navigation
  NUM=$((++NUM))
  tmux new-window -a -t $SESSION_NAME -n "Bug2_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch cocktailparty_algorithm bug2.launch robot_name:=tb3_${X}" C-m

  X=$((X + 1))
done


# attach to the tmux session
tmux attach
