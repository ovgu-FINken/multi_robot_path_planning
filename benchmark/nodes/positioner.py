#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Node responsible for re-positioning the spawned robots.
@todo:
------------------------------------------------------------- """


import rospy
import waypoint as wp
from std_msgs.msg import Int16MultiArray
import topic_handler
from geometry_msgs.msg import Point


poses = []


def callback_poses(data, args):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global poses
    poses = data.data


topic_handler.SubscribingHandler("formation", Int16MultiArray, callback_poses)

