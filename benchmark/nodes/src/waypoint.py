#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Responsible for handling way points.
@todo:
------------------------------------------------------------- """

from enum import Enum
import rospy
from geometry_msgs.msg import Point
import src.utils.topic_handler as topic_handler
import json
import os


TOPIC_NAME = "waypoint"
NODE_NAME = "waypoint_controller"
DEFAULT_NODE_UPDATE_FREQUENCY = 3


class WayPointMap(Enum):
    """ Enum of supported way point maps.
    """
    TB3_EDGE = "tb3_edge"
    MAZE = "maze"
    WAREHOUSE = "warehouse"


def get_waypoint_map(waypoint_map_name):
    """ Returns the corresponding waypoint array.
    :param waypoint_map_name:
    :return: array
    """
    dir_path = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(dir_path, '..', '..', 'config', 'waypoints.json')
    file = open(file_path)
    content = json.load(file)
    if type(waypoint_map_name) is not str:
        waypoint_map_name = waypoint_map_name.value
    try:
        wps = content.get(waypoint_map_name)
    except KeyError:
        wps = None
    return wps


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

    def __init__(self, namespace, number_of_robots,
                 waypoints=WayPointMap.TB3_EDGE,
                 node_update_frequency=DEFAULT_NODE_UPDATE_FREQUENCY,
                 callback=None, threshold=0.2):
        """ Init. method.
        :param namespace
        :param threshold:
        :param number_of_robots
        :param waypoints
        :param node_update_frequency
        :param callback: Use own callback method instead of local publisher.
        """
        self._threshold = threshold
        self._waypoint_map = get_waypoint_map(waypoints)
        self._namespace = namespace
        self._number_of_robots = number_of_robots
        self._target_point = {}
        self._node_update_frequency = node_update_frequency
        self._publisher = {}
        self._callback = callback
        if self._callback is None:
            self._setup_publisher()
        self._init_wps()

    def update(self, current_pos, quiet=False):
        """ Updates waypoints.
        :param current_pos:
        :param quiet:
        """
        for robot_name in range(self._number_of_robots):
            self._publish(robot_name)
            if robot_name in current_pos:
                if not quiet:
                    rospy.loginfo("Robot {0}: Current Pos: {1} Target Pos: {2}".format(
                        robot_name,
                        [round(current_pos[robot_name].x, 3), round(current_pos[robot_name].y, 3)],
                        [self._target_point[robot_name][0], self._target_point[robot_name][1]]))
                if self._wp_reached(current_pos[robot_name], self._target_point[robot_name]):
                    self.next(robot_name)
                    if not quiet:
                        rospy.loginfo("UPDATE for {0} to {1}!".format(
                            robot_name, self._target_point[robot_name]))

    def _wp_reached(self, current_pos, target_point):
        """ Checks whether the wp is reached within the threshold range or not.
        :param current_pos:
        :param target_point:
        :return True:   reached
                False:  not reached
        """
        if (current_pos.x - self._threshold) <= target_point[0] <= (current_pos.x + self._threshold) \
                and (current_pos.y - self._threshold) <= target_point[1] <= (current_pos.y + self._threshold):
            return True
        return False

    def _init_wps(self):
        """ Sets the initial start wp for every robot.
        """
        for robot_name in range(self._number_of_robots):
            self.next(robot_name)

    def _publish(self, robot_name):
        """ Publishes the target point for the robot.
        :param robot_name:
        """
        if self._callback is not None:
            self._callback(robot_name, self._target_point[robot_name])
        else:
            self._publish_target_points(self._target_point[robot_name])

    def next(self, robot_name):
        """ Returns the next waypoint for a given robot.
        :param robot_name:
        :return: waypoint
        """
        self._update_target_points(robot_name)
        next_wp = self._target_point[robot_name]
        self._publish(robot_name)
        return next_wp

    def _setup_publisher(self):
        """ Setup for the waypoint topics.
        """
        for robot_name in range(self._number_of_robots):
            topic_name = self._namespace + '_' + str(robot_name) + '/' + TOPIC_NAME
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
            for robot_name in range(self._number_of_robots):
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
