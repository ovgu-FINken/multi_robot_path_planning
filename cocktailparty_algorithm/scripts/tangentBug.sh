#!/usr/bin/env bash

#################################################
# @author:  Viviane Wolters
# @date:    04/2020
# @todo:
#################################################

PKG_NAME="cocktailparty_algorithm"
SESSION_NAME="TangentBug"
NUM=0
NUM_ROBOT=$(jq '.number_of_robots' ../../benchmark/settings/settings.json)

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
  tmux new-window -a -t $SESSION_NAME -n "TangentBug_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch cocktailparty_algorithm tangentBug.launch robot_name:=tb3_${X}" C-m

  X=$((X + 1))
done


# attach to the tmux session
tmux attach