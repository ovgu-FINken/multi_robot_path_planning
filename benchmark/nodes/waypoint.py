#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Responsible for handling way points.
@todo:
------------------------------------------------------------- """


import actionlib
from enum import Enum
import rospy
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import tf.transformations as tft
import random
import math
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Point
import topic_handler


TOPIC_NAME = "waypoint"
NODE_NAME = "waypoint_publisher"
DEFAULT_NODE_UPDATE_FREQUENCY = 3


class WayPointMap(Enum):
    """ Enum of supported way point maps.
    """
    EMPTY_WORLD = [
        [0.0, 0.0, 0.0]
    ]
    EDGE_TB3_WORLD = [
        [1.8, 0.0, 0.0],
        [-1.8, 0.0, 0.0],
        [0.0, 1.8, 0.0],
        [0.0, -1.8, 0.0]
    ]


def setup_node():
    """ Setup method for the way point node.
    Here the current way point is published.
    """
    rospy.init_node(NODE_NAME, anonymous=True)


class WayPointManager:
    """ Way point manager.

    This class is responsible for publishing
    the current target waypoint to the waypoint node.
    Therefore, each robot has its very own waypoint node.
    """

    def __init__(self, namespace, robot_names,
                 waypoints=WayPointMap.EMPTY_WORLD,
                 node_update_frequency=DEFAULT_NODE_UPDATE_FREQUENCY):
        """ Init. method.
        :param namespace
        :param robot_names
        :param waypoints
        :param node_update_frequency
        """
        if type(waypoints) is WayPointMap:
            self._waypoint_map = waypoints.value
        else:
            self._waypoint_map = waypoints

        self._namespace = namespace
        self._robot_names = robot_names
        self._target_point = {}
        self._node_update_frequency = node_update_frequency
        self._publisher = {}
        self._setup_publisher()

    def run(self):
        """ Starts the way point generating process.
        """
        self._loop()

    def _loop(self):
        """ Updates the node with the given frequency.
        """
        rate = rospy.Rate(self._node_update_frequency)
        while not rospy.is_shutdown():
            self._update_target_points()
            self._publish_target_points()
            rate.sleep()

    def _setup_publisher(self):
        """ Setup for the waypoint topics.
        """
        for robot_name in self._robot_names:
            topic_name = self._namespace + '_' + robot_name + '/' + TOPIC_NAME
            pub = topic_handler.PublishingHandler(topic_name, Point, queue_size=10)
            self._publisher[robot_name] = pub

    def _publish_target_points(self):
        """ Updates the target point for the robot
        by publishing the new waypoints to the topics.
        """
        for robot_name in self._robot_names:
            target_point = self._target_point[robot_name]
            self._publisher[robot_name].publish(target_point)

    def _update_target_points(self):
        """ Updates the target points for the robots.
        """
        for robot_name in self._robot_names:

            # initial target point
            if robot_name not in self._target_point:
                self._target_point[robot_name] = self._waypoint_map[0]

            # restart round
            elif self._target_point[robot_name] == self._waypoint_map[len(self._waypoint_map) - 1]:
                self._target_point[robot_name] = self._waypoint_map[0]

            # set next in round
            else:
                current_target_idx = self._waypoint_map.index(self._target_point[robot_name])
                target_point = self._waypoint_map[current_target_idx + 1]
                self._target_point[robot_name] = target_point
