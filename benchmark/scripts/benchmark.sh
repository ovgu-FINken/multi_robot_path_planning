#!/usr/bin/env bash

#################################################
# @author:  PathPlanners
# @date:    2019
# @brief:
# @todo:
#################################################
#
# USAGE RECOMMENDATION
#
# -> Create shortcuts like:
# -> gnome-terminal -- bash <FULL_PATH>/benchmark.sh
# -> gnome-terminal -- bash <FULL_PATH>/kill.sh
#
#################################################


ENABLE_RVIZ=False
ENABLE_RQT=False

SESSION_NAME="benchmark"
MAPPING=${MAPPING:=amcl}
NUM=0

NUMBER_OF_ROBOTS=1
#export ROBOT_POSE_X=0
#export ROBOT_POSE_Y=0

# source
source ~/.bashrc

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark world.launch world:=turtlebot3_world.world" C-m
read -t 3

# spawner setup
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "spawner_setup"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark spawner.launch" C-m

# spawner
for i in ${NUMBER_OF_ROBOTS} ; do
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "spawner_robot_${i}"
  #tmux send-keys -t $SESSION_NAME:$NUM "roscd benchmark" C-m
  #read -t 3
  #tmux send-keys -t $SESSION_NAME:$NUM "rosparam load param/robot_${i}_params.yaml" C-m
  #read -t 3
  #tmux send-keys -t $SESSION_NAME:$NUM "ROBOT_POSE_X=$(rosparam get pose_x)" C-m
  #read -t 3
  #tmux send-keys -t $SESSION_NAME:$NUM "ROBOT_POSE_Y=$(rosparam get pose_y)" C-m
  #tmux send-keys -t $SESSION_NAME:$NUM "export ROBOT_POSE_X=${ROBOT_POSE_X:=$(rosparam get pose_x)}" C-m
  #read -t 3
  #export ROBOT_POSE_Y=${ROBOT_POSE_Y:=$(tmux send-keys -t $SESSION_NAME:$NUM "rosparam get pose_y" C-m)}
  #read -t 3
  #tmux setenv ROBOT_POSE_X "$POSE_X"
  #tmux send-keys -t $SESSION_NAME:$NUM "export ROBOT_POSE_X=""$POSE_X" C-m
  #read -t 3
  #tmux send-keys -t $SESSION_NAME:$NUM "echo $ROBOT_POSE_X" C-m
  #read -t 3
  #tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark spawn.launch nr:=2 pose_x:='0.5 0.0' pose_y:='0.0 0.5'" C-m
done

# waypoints
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "waypoints"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark waypoints.launch" C-m

# movement
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "movement"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark movement.launch" C-m

# random walk
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "random"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark random.launch" C-m

# rviz
if [ $ENABLE_RVIZ == True ] ; then
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "rviz"
  tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m
fi

# rqt
if [ $ENABLE_RQT == True ] ; then
  NUM=$((++NUM))
  tmux new-window -t $SESSION_NAME -n "rqt"
  tmux send-keys -t $SESSION_NAME:$NUM "rqt" C-m
fi

# attach to the tmux session
tmux attach
