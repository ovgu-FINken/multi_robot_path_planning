#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     These methods are responsible for the
            movement of the robot.
@todo:
------------------------------------------------------------- """


import actionlib
import math
import random
import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


class MovementController:
    """ Movement controller.
    """

    def __init__(self):
        """ Init. method.
        """
        self._client = None
        self._setup_action_client()

    def _setup_action_client(self):
        """ Setup for action client.
        """
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()
        self._client = client

    def linear_move_to(self, goal_pos, execute_timeout=5, quiet=True):
        """ Moves the robot straight to the goal position,
        :param goal_pos:
        :param execute_timeout:
        :param quiet
        :return True    successful
                False   goal is None
        """
        goal = MoveBaseGoal()
        goal.target_pose.pose.position.x = goal_pos[0]
        goal.target_pose.pose.position.y = goal_pos[1]
        if not quiet:
            rospy.loginfo("Moving to goal {}.".format(goal))
        if goal is None:
            return False
        self._client.send_goal_and_wait(
            goal, execute_timeout=rospy.Duration.from_sec(execute_timeout))
        return True
