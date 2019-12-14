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
from std_msgs.msg import Bool
import src.timer as time
import src.utils.log as log


def callback_target(data, args):
    """ Callback when a wp is set.
    :param data:
    :param args:
    """
    global timer, previous_wps, logger, makespan
    if args[0] not in previous_wps:
        previous_wps[args[0]] = data
        timer.start_timer(args[0])
        makespan.start_timer(args[0])
    elif previous_wps[args[0]] != data:
        if timer.is_running(args[0]):
            duration = timer.get_time(args[0])
            logger.time(
                namespace + str(args[0]),
                [previous_wps[args[0]].x, previous_wps[args[0]].y, previous_wps[args[0]].z],
                [data.x, data.y, data.z], str(duration) + "s")
            print("Robot {0} reached WP ({1}) after {2}s".format(
                args[0], [data.x, data.y, data.z], duration))
        previous_wps[args[0]] = data
        timer.start_timer(args[0])


def callback_rounds(data, args):
    """ Callback when the rounds are fulfilled.
    :param data:
    :param args:
    """
    global logger, makespan
    if data:
        print("Robot {0} finished in makespan of {1}s".format(
            args[0], makespan.get_time(args[0])))
        logger.makespan(args[0], makespan.get_time(args[0]))


def setup_subscriber(_number_of_robots, _namespace):
    """ Setup for subscriber.
    :param _number_of_robots:
    :param _namespace:
    """
    for robot_id in range(number_of_robots):
        topic_name = namespace + str(robot_id) + "/waypoint"
        topic_handler.SubscribingHandler(topic_name, Point, callback_target, robot_id)
        topic_name = namespace + str(robot_id) + "/rounds"
        topic_handler.SubscribingHandler(topic_name, Bool, callback_rounds, robot_id)


logger = log.Logger()
previous_wps = {}
rospy.init_node('evaluation_controller', anonymous=True)
namespace = rospy.get_param('namespace')
number_of_robots = rospy.get_param('number_of_robots')
timer = time.Timer(number_of_robots)
makespan = time.Timer(number_of_robots)
setup_subscriber(number_of_robots, namespace)
rospy.spin()
