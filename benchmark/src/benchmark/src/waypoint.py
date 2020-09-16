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
from std_msgs.msg import Bool
import json
import os
import copy
import src.utils.naming_scheme as names


DEFAULT_NODE_UPDATE_FREQUENCY = 3


class WayPointMap(Enum):
    """ Enum of supported way point maps.
    """
    TB3_EDGE = "tb3_edge"
    MAZE = "maze"
    WAREHOUSE = "warehouse"
    TWO_ROOMS = "two_rooms"


def get_num_of_wps(waypoint_map_name):
    """ Returns the number of wps in a wp map.
    :param waypoint_map_name:
    :return: number (int)
    """
    wp_dict = get_waypoint_map(waypoint_map_name)
    if wp_dict is None:
        return 0
    return len(wp_dict)


def get_waypoint_map(waypoint_map_name):
    """ Returns the corresponding waypoint array.
    :param waypoint_map_name:
    :return: array
    """
    dir_path = os.path.dirname(os.path.realpath(__file__))
    file_path = os.path.join(dir_path, '..', '..', '..', 'config', 'waypoints.json')
    file = open(file_path)
    content = json.load(file) # returns dict
    if type(waypoint_map_name) is not str:
        waypoint_map_name = waypoint_map_name.value
    try:
        wps = content.get(waypoint_map_name) # returns a list with tupels
        for wp in wps:
            rospy.loginfo("Received waypoints: %f, %f, %f", wp[0], wp[1], wp[2])
    except KeyError:
        wps = None
        rospy.logerr("Waypoints could not be read.")
    return wps


def setup_node():
    """ Setup method for the way point node.
    Here the current way point is published.
    """
    rospy.init_node(names.NodeNames.WAYPOINT_CONTROLLER.value, anonymous=True)

def point_in_square(point, cp, r):
    if point.x > (cp[0] - r):
        if point.x < (cp[0] + r):
            if point.y > (cp[1] - r):
                if point.y < (cp[1] + r):
                    return True
    return False

class WayPointManager:
    """ Way point manager.

    This class is responsible for publishing
    the current target waypoint to the waypoint node.
    Therefore, each robot has its very own waypoint node.
    """

    def __init__(self, namespace, number_of_robots, rounds, start_positions,
                 waypoints=WayPointMap.TB3_EDGE,
                 node_update_frequency=DEFAULT_NODE_UPDATE_FREQUENCY,
                 wp_callback=None, finished_callback=None,
                 threshold=0.2):
        """ Init. method.
        :param rounds:
        :param namespace
        :param threshold:
        :param number_of_robots
        :param waypoints
        :param node_update_frequency
        :param finished_callback:
        :param wp_callback: Use own wp_callback method instead of local publisher.
        """
        self._rounds = rounds
        self._threshold = threshold
        self._waypoint_map = get_waypoint_map(waypoints)
        self._namespace = namespace
        self._number_of_robots = number_of_robots
        self._target_point = {}
        self._node_update_frequency = node_update_frequency
        self._publisher = {}
        self._wp_callback = wp_callback
        self._finished_callback = finished_callback
        self._rounds_completed = {}
        self._waypoint_map_name = waypoints
        self.publish_start_positions = start_positions

        if self._waypoint_map_name == "two_rooms":
            self._waypoint_map = self.make_different_maps(start_positions)
            print(self._waypoint_map)
            
        self._setup_publisher()
        self._init_wps()


    def update(self, current_pos, quiet=False):
        """ Updates waypoints.
        :param current_pos:
        :param quiet:
        """
        for robot_name in range(self._number_of_robots):
            if robot_name in current_pos:
                if not quiet:
                    self._print_pos(robot_name, current_pos)
                if self._wp_reached(current_pos[robot_name], self._get_target_point(robot_name)):
                    self.next(robot_name)
                    if not quiet:
                        rospy.loginfo("UPDATE for {0} to {1}!".format(
                            robot_name, self._get_target_point(robot_name)))
            self._publish(robot_name)

    def _print_pos(self, robot_name, current_pos):
        """ prints the current position and the target position in the console.
        :param robot_name:
        :param current_pos:
        """
        target = self._get_target_point(robot_name)
        if target is None or len(target) <= 0:
            return
        rospy.loginfo("Robot {0}: Current Pos: {1} Target Pos: {2}".format(
            robot_name,
            [round(current_pos[robot_name].x, 3), round(current_pos[robot_name].y, 3)],
            [target[0], target[1]]))

    def _get_target_point(self, robot_name):
        """ Returns the current target point for the robot.
        :param robot_name:
        :return target point
        """
        if robot_name not in self._target_point.keys():
            return []
        return self._target_point[robot_name][-1]

    def _set_target_point(self, robot_name, target_point):
        """ Sets the newest target point for a robot.
        :param robot_name:
        :param target_point:
        """
        if robot_name not in self._target_point:
            self._target_point[robot_name] = []
        self._target_point[robot_name].append(target_point)

    def _finished(self, robot_name):
        """ Checks whether the robot has finished the benchmark.
        :param robot_name
        :return boolean
        """
        if robot_name not in range(self._number_of_robots):
            return False
        if robot_name not in self._target_point.keys():
            return False
        # The count() is a built-in function in Python. It will return you the count of a given element in the list.
        if self._waypoint_map_name == "two_rooms":
           if self._target_point[robot_name].count(self._waypoint_map[robot_name][1]) - 1 >= self._rounds:
               return True
        else:        
           if self._target_point[robot_name].count(self._waypoint_map[1]) - 1 >= self._rounds:
               return True
        return False

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
        if self._wp_callback is not None:
            self._wp_callback(robot_name, self._get_target_point(robot_name))
            self._finished_callback(robot_name, self._finished(robot_name))
        else:
            self._publish_target_points(robot_name)
            self._publish_rounds(robot_name)

    def next(self, robot_name): 
        """ Returns the next waypoint for a given robot.
        :param robot_name:
        :return: waypoint
        """
        self._update_target_points(robot_name) 
        next_wp = self._get_target_point(robot_name)
        self._publish(robot_name)
        return next_wp

    def _setup_publisher(self):
        """ Setup for the waypoint topics.
        """
        if self._wp_callback is None:
            self._publisher[names.TopicNames.WAYPOINT.value] = {}
            for robot_name in range(self._number_of_robots):
                topic_name = self._namespace + '_' + str(robot_name) + '/' + names.TopicNames.WAYPOINT.value
                pub = topic_handler.PublishingHandler(topic_name, Point, queue_size=10)
                self._publisher[names.TopicNames.WAYPOINT.value][robot_name] = pub
        if self._finished_callback is None:
            self._publisher[names.TopicNames.FINISHED.value] = {}
            for robot_name in range(self._number_of_robots):
                topic_name = self._namespace + '_' + str(robot_name) + '/' + names.TopicNames.FINISHED.value
                pub = topic_handler.PublishingHandler(topic_name, Bool, queue_size=10)
                self._publisher[names.TopicNames.FINISHED.value][robot_name] = pub

    def _publish_target_points(self, robot_name=None):
        """ Updates the target point for the robot
        by publishing the new waypoints to the topics.
        :param robot_name: publish only for this robot
        """
        if robot_name is not None:
            self._publisher[names.TopicNames.WAYPOINT.value][robot_name].publish(self._get_target_point(robot_name))
        else:
            for robot_name in range(self._number_of_robots):
                target_point = self._get_target_point(robot_name)
                self._publisher[names.TopicNames.WAYPOINT.value][robot_name].publish(target_point)

    def _publish_rounds(self, robot_name=None):
        """ Updates the rounds for the robot
        by publishing the boolean to the topics.
        :param robot_name: publish only for this robot
        """
        if robot_name is not None:
            self._publisher[names.TopicNames.FINISHED.value][robot_name].publish(self._get_target_point(robot_name))
        else:
            for robot_name in range(self._number_of_robots):
                target_point = self._get_target_point(robot_name)
                self._publisher[names.TopicNames.FINISHED.value][robot_name].publish(target_point)

    #change here
    def _update_target_points(self, robot_name):
        """ Updates the target points for a robot.
        """

        # initial target point
        if robot_name not in self._target_point:
            rospy.loginfo("Setting intial target point...")
            if self._waypoint_map_name == "two_rooms":
                self._set_target_point(robot_name, self._waypoint_map[robot_name][0]) 
            else:    
                self._set_target_point(robot_name, self._waypoint_map[0]) 
            rospy.loginfo("Intial target point is set.")

        # restart round
        elif (self._get_target_point(robot_name) == self._waypoint_map[-1] or self._get_target_point(robot_name) == self._waypoint_map[robot_name][-1]):
            rospy.loginfo("Restarting round...")
            if self._waypoint_map_name == "two_rooms":
                self._set_target_point(robot_name, self._waypoint_map[robot_name][0]) 
            else:    
                self._set_target_point(robot_name, self._waypoint_map[0]) 
            rospy.loginfo("Round restarted.")

        # set next in round
        else:
            rospy.loginfo("Setting next target in round...")
            if self._waypoint_map_name == "two_rooms":
                current_target_idx = self._waypoint_map[robot_name].index(self._get_target_point(robot_name))
                target_point = self._waypoint_map[robot_name][current_target_idx + 1]
            else:    
                current_target_idx = self._waypoint_map.index(self._get_target_point(robot_name))
                target_point = self._waypoint_map[current_target_idx + 1]
            self._set_target_point(robot_name, target_point)
            rospy.loginfo("Next target in round is set.")



##### JUST FOR TWO ROOM SCENARIO #####

    def make_different_maps(self, start_positions):

        self.mapA = copy.deepcopy(self._waypoint_map)

        self.mapB = copy.deepcopy(self.mapA)
        self.mapB.reverse()

        self.mapC = copy.deepcopy(self.mapA)
        for i in range(len(self.mapC)):
            self.mapC[i][0] *= -1

        self.mapD = copy.deepcopy(self.mapC)
        self.mapD.reverse()


        maps = []
        cpA= [-1.0, -1.0]
        cpB= [1.0, 2.0]
        cpC= [1.0, -1.0]
        cpD= [-1.0, 2.0]

        for i in range(self._number_of_robots):
            if point_in_square(point=start_positions[i], cp=cpA, r=1.0):
                # self.mapA = cpA + self.mapA
                maps.append(self.mapA)

            if point_in_square(point=start_positions[i], cp=cpB, r=1.0):
                # self.mapB = cpB + self.mapB
                maps.append(self.mapB)

            if point_in_square(point=start_positions[i], cp=cpC, r=1.0):
                # self.mapC = cpC + self.mapC
                maps.append(self.mapC)

            if point_in_square(point=start_positions[i], cp=cpD, r=1.0):
                # self.mapD = cpD + self.mapD
                maps.append(self.mapD)

        return maps
