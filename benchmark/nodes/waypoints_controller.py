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


def callback_target(name, point):
    """ This callback method will publish the new target points.
    :param name:
    :param point:
    """
    global publisher
    publisher[name].publish(point, quiet=True)


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


def setup_odometry_subscriber(_number_of_robots, _namespace):
    """ Init for the waypoint publisher.
    :param _number_of_robots:
    :param _namespace:
    """
    for robot_id in range(_number_of_robots):
        odom_name = _namespace + str(robot_id) + "/odom"
        topic_handler.SubscribingHandler(odom_name, Odometry, callback_odometry, robot_id)


def update_wps(_number_of_robots, _namespace, _wp_map, frequency=0.5):
    """ Updates the waypoints for the robots.
    :param _number_of_robots:
    :param _wp_map:
    :param _namespace:
    :param frequency:
    """
    wp_manager = wp.WayPointManager(
        namespace=_namespace, number_of_robots=_number_of_robots,
        callback=callback_target, waypoints=_wp_map)
    while not rospy.is_shutdown():
        wp_manager.update(robot_current_positions)
        rospy.Rate(frequency).sleep()


robot_current_positions = {}
publisher = {}
rospy.init_node("waypoint_controller", anonymous=True)
namespace = rospy.get_param('namespace')
wp_map = rospy.get_param('wp_map')
number_of_robots = rospy.get_param('number_of_robots')
setup_waypoint_publisher(publisher, number_of_robots, namespace)
setup_odometry_subscriber(number_of_robots, namespace)
update_wps(number_of_robots, namespace, wp_map)
