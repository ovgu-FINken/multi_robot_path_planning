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
NODE_NAME = "waypoint_controller"
DEFAULT_NODE_UPDATE_FREQUENCY = 3


class WayPointMap(Enum):
    """ Enum of supported way point maps.
    """
    EMPTY_WORLD = "empty_world"
    EDGE_TB3_WORLD = "edge_tb3_world"


def get_waypoint_map(waypoint_map_name):
    """ Returns the corresponding waypoint array.
    :param waypoint_map_name:
    :return: array
    """
    if waypoint_map_name == WayPointMap.EMPTY_WORLD \
            or waypoint_map_name == WayPointMap.EMPTY_WORLD.value:
        return [[0.0, 0.0, 0.0]]
    elif waypoint_map_name == WayPointMap.EDGE_TB3_WORLD \
            or waypoint_map_name == WayPointMap.EDGE_TB3_WORLD.value:
        return [[1.8, 0.0, 0.0],
                [-1.8, 0.0, 0.0],
                [0.0, 1.8, 0.0],
                [0.0, -1.8, 0.0]]
    else:
        rospy.logerr("Unknown waypoint map name {}".format(waypoint_map_name))
        return None


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
                 node_update_frequency=DEFAULT_NODE_UPDATE_FREQUENCY,
                 callback=None):
        """ Init. method.
        :param namespace
        :param robot_names
        :param waypoints
        :param node_update_frequency
        :param callback: Use own callback method instead of local publisher.
        """
        self._waypoint_map = get_waypoint_map(waypoints)
        self._namespace = namespace
        self._robot_names = robot_names
        self._target_point = {}
        self._node_update_frequency = node_update_frequency
        self._publisher = {}
        self._callback = callback
        if self._callback is None:
            self._setup_publisher()

    def update(self, frequency=1):
        """ Updates waypoints.
        :param frequency
        """
        wps = []
        for robot_name in self._robot_names:
            self._update_target_points(robot_name)
            wps.append(self._target_point[robot_name])
        while not rospy.is_shutdown():
            for r, robot_name in enumerate(self._robot_names):
                if self._callback is not None:
                    self._callback(robot_name, wps[r])
                else:
                    self._publish_target_points(wps[r])
            rospy.Rate(frequency).sleep()

    def next(self, robot_name):
        """ Returns the next waypoint for a given robot.
        :param robot_name:
        :return: waypoint
        """
        self._update_target_points(robot_name)
        next_wp = self._target_point[robot_name]
        if self._callback is not None:
            self._callback(robot_name, next_wp)
        else:
            self._publish_target_points(next_wp)
        return next_wp

    def _setup_publisher(self):
        """ Setup for the waypoint topics.
        """
        for robot_name in self._robot_names:
            topic_name = self._namespace + '_' + robot_name + '/' + TOPIC_NAME
            pub = topic_handler.PublishingHandler(topic_name, Point, queue_size=10)
            self._publisher[robot_name] = pub

    def _publish_target_points(self, robot_name=None):
        """ Updates the target point for the robot
        by publishing the new waypoints to the topics.
        :param robot_name: publish only for this robot
        """
        if robot_name is not None:
            self._publisher[robot_name].publish(self._target_point[robot_name])
        else:
            for robot_name in self._robot_names:
                target_point = self._target_point[robot_name]
                self._publisher[robot_name].publish(target_point)

    def _update_target_points(self, robot_name):
        """ Updates the target points for a robot.
        """
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
