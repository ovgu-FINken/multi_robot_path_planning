#!/usr/bin/env bash

##################################################################################################
# @author:  Kilian Poessel
# @date:    2020/05/06
# @brief:   Executes the multi-robot navigation with additional costmap layer for robots' paths.
# @todo:
##################################################################################################

# source
#source ~/.bashrc

#### script parameters
SESSION_NAME="pathcostmap"
NUM=0
MAPPING=${MAPPING:=amcl}
WORLD=""
NUM_ROBOT_DEFAULT=3