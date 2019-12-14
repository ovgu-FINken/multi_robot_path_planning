#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Controller between launch file and python files and nodes.
@todo:
------------------------------------------------------------- """


import rospy
import src.waypoint as wp
from std_msgs.msg import Int16MultiArray
from nav_msgs.msg import Odometry
import src.utils.topic_handler as topic_handler
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


def callback_target(name, point):
    """ This callback method will publish the new target points.
    :param name:
    :param point:
    """
    global target_publisher
    if point is not None and len(point) > 0:
        target_publisher[name].publish(point, quiet=True)


def callback_rounds(name, finished):
    """ This callback method will publish the new round status.
    :param name:
    :param finished:
    """
    global rounds_publisher
    rounds_publisher[name].publish(finished, quiet=True)


def callback_odometry(data, args):
    """ This callback method will save the current position of the robots.
    :param data:
    :param args:
    """
    global robot_current_positions
    robot_current_positions[args[0]] = data.pose.pose.position


def setup_waypoint_publisher(_publisher, _number_of_robots, _namespace):
    """ Init for the waypoint publisher.
    :param _publisher:
    :param _number_of_robots:
    :param _namespace:
    """
    for robot_id in range(_number_of_robots):
        _topic_name = _namespace + str(robot_id) + "/waypoint"
        _pub = topic_handler.PublishingHandler(_topic_name, Point, queue_size=10)
        _publisher[robot_id] = _pub


def setup_rounds_publisher(_publisher, _number_of_robots, _namespace):
    """ Init for the rounds publisher.
    :param _publisher:
    :param _number_of_robots:
    :param _namespace:
    """
    for robot_id in range(_number_of_robots):
        _topic_name = _namespace + str(robot_id) + "/rounds"
        _pub = topic_handler.PublishingHandler(_topic_name, Bool, queue_size=10)
        _publisher[robot_id] = _pub


def setup_odometry_subscriber(_number_of_robots, _namespace):
    """ Init for the waypoint publisher.
    :param _number_of_robots:
    :param _namespace:
    """
    for robot_id in range(_number_of_robots):
        odom_name = _namespace + str(robot_id) + "/odom"
        topic_handler.SubscribingHandler(odom_name, Odometry, callback_odometry, robot_id)


def update_wps(_number_of_robots, _namespace, _rounds,
               _wp_map, _wp_threshold, frequency=0.5):
    """ Updates the waypoints for the robots.
    :param _number_of_robots:
    :param _wp_map:
    :param _rounds:
    :param _namespace:
    :param frequency:
    :param _wp_threshold:
    """
    wp_manager = wp.WayPointManager(
        namespace=_namespace, number_of_robots=_number_of_robots,
        wp_callback=callback_target, round_callback=callback_rounds,
        waypoints=_wp_map, threshold=wp_threshold, rounds=_rounds)
    while not rospy.is_shutdown():
        wp_manager.update(robot_current_positions)
        rospy.Rate(frequency).sleep()


robot_current_positions = {}
target_publisher = {}
rounds_publisher = {}
rospy.init_node("waypoint_controller", anonymous=True)
namespace = rospy.get_param('namespace')
wp_map = rospy.get_param('wp_map')
wp_threshold = rospy.get_param('wp_threshold')
number_of_robots = rospy.get_param('number_of_robots')
rounds = rospy.get_param('rounds')
setup_waypoint_publisher(target_publisher, number_of_robots, namespace)
setup_rounds_publisher(rounds_publisher, number_of_robots, namespace)
setup_odometry_subscriber(number_of_robots, namespace)
update_wps(number_of_robots, namespace, rounds, wp_map, wp_threshold)
