#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Controller between launch file and python files and nodes.
@todo:
------------------------------------------------------------- """


import rospy
import spawner as sp
import waypoint as wp
import formation as form


DEFAULT_MODEL_NAME = "turtlebot3"
DEFAULT_MODEL_TYPE = "burger"
DEFAULT_NUMBER_OF_ROBOTS = 3
DEFAULT_POSITION = [0, 0, 0]
DEFAULT_ORIENTATION = [0, 0, 0]
DEFAULT_NAME = "0"
DEFAULT_NAMESPACE = "tb3_"
DEFAULT_FORMATION = form.Formation.DENSE_BLOCK


wp.setup_node()

namespace = rospy.get_param('~namespace', DEFAULT_NAMESPACE)

wp_manager = wp.WayPointManager(namespace=namespace)

