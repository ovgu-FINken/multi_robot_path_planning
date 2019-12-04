#!/usr/bin/env bash

#################################################
# @author:  PathPlanners
# @date:    2019
# @brief:
# @todo:
#################################################


# user parameters
ENABLE_RVIZ=False
ENABLE_RQT=False
NUMBER_OF_ROBOTS=4
NAMESPACE="tb3_"
MODEL="turtlebot"
MODEL_TYPE="burger"

# script parameters
SESSION_NAME="benchmark"
NUM=0

# launch file parameters
PARAM_NUMBER_OF_ROBOTS="number_of_robots:=$NUMBER_OF_ROBOTS"
PARAM_NAMESPACE="namespace:=$NAMESPACE"
PARAM_MODEL_TYPE="model_type:=$MODEL_TYPE"

# source
source ~/.bashrc

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark world.launch world:=turtlebot3_world.world" C-m

# spawner
NUM=$((++NUM))
PARAMS="$PARAM_NUMBER_OF_ROBOTS $PARAM_NAMESPACE $PARAM_MODEL_TYPE"
tmux new-window -t $SESSION_NAME -n "spawner"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark spawner.launch $PARAMS" C-m

# waypoints
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "waypoints"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark waypoints.launch" C-m

# movement
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "movement"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch benchmark movement.launch" C-m

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
