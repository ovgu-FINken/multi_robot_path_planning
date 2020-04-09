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
import src.waypoint as waypoint
import src.utils.naming_scheme as names


def callback_target(data, args):
    """ Callback when a wp is set.
    :param data:
    :param args:
    """
    global timer, previous_wps, logger, makespan, end
    if not end:
        if args[0] not in previous_wps:
            previous_wps[args[0]] = data
            if include_start_time:
                timer.start_timer(args[0])
                makespan.start_timer(args[0])
        elif previous_wps[args[0]] != data:
            if timer.is_running(args[0]):
                duration = timer.get_time(args[0])
                logger.wptime(
                    namespace + str(args[0]),
                    [previous_wps[args[0]].x, previous_wps[args[0]].y, previous_wps[args[0]].z],
                    [data.x, data.y, data.z], str(duration) + "s")
            previous_wps[args[0]] = data
            timer.start_timer(args[0])


def callback_finished(data, args):
    """ Callback when the rounds are fulfilled.
    :param data:
    :param args:
    """
    global logger, makespan, finished, end
    if not end:
        if data == Bool(True) and not finished[args[0]]:
            logger.makespan(namespace + str(args[0]), makespan.get_time(args[0]))
            flowtime = makespan.get_time(args[0]) / waypoint.get_num_of_wps(wp_map)
            logger.flowtime(namespace + str(args[0]), flowtime)
            finished[args[0]] = True
        if all(item is True for item in finished.values()):
            makespan_list = [makespan.get_time(key) for key in range(number_of_robots)]
            makespan_avg = sum(makespan_list) / len(makespan_list)
            logger.makespan_avg(makespan_avg)
            flowtime_avg = makespan_avg / waypoint.get_num_of_wps(wp_map)
            logger.flowtime_avg(flowtime_avg)
            end = True


def setup_subscriber(_number_of_robots, _namespace):
    """ Setup for subscriber.
    :param _number_of_robots:
    :param _namespace:
    """
    global finished
    for robot_id in range(number_of_robots):
        finished[robot_id] = False
        topic_name = namespace + str(robot_id) + "/" + names.TopicNames.WAYPOINT.value
        topic_handler.SubscribingHandler(topic_name, Point, callback_target, robot_id)
        topic_name = namespace + str(robot_id) + "/" + names.TopicNames.FINISHED.value
        topic_handler.SubscribingHandler(topic_name, Bool, callback_finished, robot_id)


logger = log.Logger()
previous_wps = {}
finished = {}
flowtime = {}
end = False
rospy.init_node(names.NodeNames.EVALUATION_CONTROLLER.value, anonymous=True)
namespace = rospy.get_param('namespace')
number_of_robots = rospy.get_param('number_of_robots')
include_start_time = rospy.get_param('include_start_time')
wp_map = rospy.get_param('wp_map')
timer = time.Timer(number_of_robots)
makespan = time.Timer(number_of_robots)
setup_subscriber(number_of_robots, namespace)
rospy.spin()
