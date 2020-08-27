#!/usr/bin/env bash

#################################################
# @author:  Iffat Jamil
# @date:    Aug/2020
# @todo:
#################################################

PKG_NAME="vo_based_algo"
SESSION_NAME="Vo_Based_Algo"
NUM=0
NUM_ROBOT=4

source ~/.bashrc

# start tmux
tmux new-session -s $SESSION_NAME -d

X=0
while [ $X -lt $NUM_ROBOT ]; do

  # #localisation
  # NUM=$((++NUM))
  # tmux new-window -t $SESSION_NAME -n "amcl_${X}"
  # tmux send-keys -t $SESSION_NAME:$NUM "roslaunch vo_based_algo amcl.launch robot:=tb3_${X}" C-m


  # navigation
  NUM=$((++NUM))
  tmux new-window -a -t $SESSION_NAME -n "vo_based_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch vo_based_algo vo_based.launch robot_name:=tb3_${X}" C-m

  X=$((X + 1))
done


# attach to the tmux session
tmux attach
