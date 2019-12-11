#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019
@brief:     Creates a gazebo world.
@todo:
------------------------------------------------------------- """


import rospy
import os


rospy.init_node('world_creator', anonymous=True)
world_name = rospy.get_param('world')
dir_path = os.path.dirname(os.path.realpath(__file__))
world_path = os.path.join(dir_path, '..', 'worlds', world_name)
os.system("roslaunch benchmark empty_world.launch world_name:=" + world_path)
