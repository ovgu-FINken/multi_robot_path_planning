#!/usr/bin/env bash

#################################################
# @author:  Iffat Jamil
# @date:    Aug/2020
# @todo:
#################################################

PKG_NAME="rvo_turtlebot3"
SESSION_NAME="Vo_Based"
NUM=0
NUM_ROBOT=8

source ~/.bashrc

# start tmux

tmux new-session -s $SESSION_NAME -d

# Launch Gazebo

tmux new-window -t $SESSION_NAME -n "Running_gazebo"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch" C-m

#wait for few sec

sleep 30

# Circle Formation

tmux new-window -t $SESSION_NAME -n "circle_formation"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch rvo_turtlebot3 vo_based_algo_version_2.launch" C-m

#wait for few sec

sleep 5

# Goal Distribution
X=0
while [ $X -le $NUM_ROBOT ]; 

do

  # navigation
  NUM=$((++NUM))
  tmux new-window -a -t $SESSION_NAME -n "Vo_Based_${X}"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch rvo_turtlebot3 distributing_goals.launch agent_name:=tb3_${X}" c-m

  X=$((X + 1))
done


# attach to the tmux session
tmux attach
