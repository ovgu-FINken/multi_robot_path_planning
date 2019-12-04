#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019
@brief:     This controller is responsible for measuring,
            and evaluating the performance of the robots.
@todo:
------------------------------------------------------------- """


import rospy
import src.utils.topic_handler as topic_handler
from geometry_msgs.msg import Point
import src.timer as time


def callback_target(data, args):
    """ Callback when a new wp is set.
    :param data:
    :param args:
    """
    global timer
    timer.start_timer(args[0])


def setup_subscriber(_number_of_robots, _namespace):
    """ Setup for subscriber.
    :param _number_of_robots:
    :param _namespace:
    """
    for robot_id in range(number_of_robots):
        topic_name = namespace + str(robot_id) + "/waypoint"
        topic_handler.SubscribingHandler(topic_name, Point, callback_target, robot_id)


rospy.init_node('evaluation_controller', anonymous=True)
namespace = rospy.get_param('namespace')
number_of_robots = rospy.get_param('number_of_robots')
timer = time.Timer(number_of_robots)
setup_subscriber(number_of_robots, namespace)
