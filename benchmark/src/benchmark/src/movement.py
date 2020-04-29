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
import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from math import pow, atan2, sqrt
import tf.transformations as tft


class MovementController:
    """ Movement controller.
    """

    def __init__(self, robot_name, namespace, waiting_time):
        """ Init. method.
        :param robot_name
        :param namespace
        """
        self._robot_name = robot_name
        self._namespace = namespace
        self._client = None
        self._setup_action_client(quiet=False, duration=waiting_time)

    def _setup_action_client(self, quiet=True, duration=0):
        """ Setup for action client.
        :param quiet
        """
        topic = self._namespace + self._robot_name + "/move_base"
        client = actionlib.SimpleActionClient(topic, MoveBaseAction)
        if not quiet:
            rospy.loginfo("Initializing move base action client ...")
            rospy.loginfo("Client for topic " + topic)
        if (client.wait_for_server(rospy.Duration(duration))):
            if not quiet:
                rospy.loginfo(
                    "Move base action client setup finished successfully!")
            self._client = client
        else:
            rospy.logerr(
                "Connection to action server failed. Action client setup failed.")

    def move_to(self, goal_pos, execute_timeout=5, quiet=True):
        """ Moves the robot straight to the goal position,
        :param goal_pos:
        :param execute_timeout:
        :param quiet
        :return True    successful
                False   goal is None
        """
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()  # - rospy.Duration(0.1)
        goal.target_pose.pose.position.x = goal_pos[0]
        goal.target_pose.pose.position.y = goal_pos[1]
        goal.target_pose.pose.position.z = goal_pos[2]
        angle = random.uniform(-math.pi, math.pi)
        rotation = tft.quaternion_from_euler(angle, 0, 0)
        # goal.target_pose.pose.orientation = rotation
        # goal.target_pose.pose.orientation.x = rotation[0]
        # goal.target_pose.pose.orientation.y = rotation[1]
        # goal.target_pose.pose.orientation.z = rotation[2]
        goal.target_pose.pose.orientation.w = 1.0  # rotation[3]

        if not quiet:
            rospy.loginfo("Moving Frame ID {}.".format(
                goal.target_pose.header.frame_id))
            rospy.loginfo("Moving to goal position {}.".format(goal_pos))
        if goal is None:
            return False
        self._client.send_goal(goal)
        rospy.loginfo("Forward goal: %f, %f, %f", goal.target_pose.pose.position.x,
                      goal.target_pose.pose.position.y, goal.target_pose.pose.position.z)
        return True
