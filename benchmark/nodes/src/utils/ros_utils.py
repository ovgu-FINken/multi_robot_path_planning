#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Fundamental utility methods for ROS.
@todo:
------------------------------------------------------------- """


import sys
import roslib
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest, SpawnModelRequest
import rospy
import os
from geometry_msgs.msg import *
import tf.transformations as tft
from std_msgs.msg import Empty as EmptyMsg
from geometry_msgs.msg import Quaternion
import math


def get_obj_pose(position, orientation):
    """ Returns the pose of the object.
    :param position:
    :param orientation:
    :return: pose
    """
    quaternion = tft.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    object_pose = Pose()
    object_pose.position.x = float(position[0])
    object_pose.position.y = float(position[1])
    object_pose.position.z = float(position[2])
    object_pose.orientation.x = quaternion[0]
    object_pose.orientation.y = quaternion[1]
    object_pose.orientation.z = quaternion[2]
    object_pose.orientation.w = quaternion[3]
    return object_pose

