#!/usr/bin/env bash

#################################################
# @author:  PathPlanners
# @date:    2019
# @todo:
#################################################

SESSION_NAME="multi_robo"
MAPPING=${MAPPING:=amcl}
NUM=0
TERMINAL="gnome-terminal"

# source
source ~/.bashrc
#source ~/git/DrivingSwarm/devel/local_setup.bash

# start tmux
tmux new-session -s $SESSION_NAME -d

# open world
tmux rename-window -t $SESSION_NAME "world"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot simulation.launch world:=turtlebot3_world.world" C-m
read -t 3

# mapping 0
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "mapping 0"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot $MAPPING.launch robot_name:=tb3_0" C-m

# mapping 1
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "mapping 1"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot $MAPPING.launch robot_name:=tb3_1" C-m

# mapping 2
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "mapping 2"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot $MAPPING.launch robot_name:=tb3_2" C-m
read -t 3

# follower
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "follower"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot follower.launch" C-m

# leader
#NUM=$((++NUM))
#tmux new-window -t $SESSION_NAME -n "leader"
#tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot leader.launch" C-m

# leader teleop
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "leader teleop"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot teleop_key.launch" C-m

# random walk
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "random"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch multi_robot random.launch" C-m

# rviz
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rviz"
tmux send-keys -t $SESSION_NAME:$NUM "roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch" C-m

# rqt
NUM=$((++NUM))
tmux new-window -t $SESSION_NAME -n "rqt"
tmux send-keys -t $SESSION_NAME:$NUM "rqt" C-m

# attach to the tmux session
tmux attach
