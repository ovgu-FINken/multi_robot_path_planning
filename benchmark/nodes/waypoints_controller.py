#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Controller between launch file and python files and nodes.
@todo:
------------------------------------------------------------- """


import rospy
import waypoint as wp
from std_msgs.msg import Int16MultiArray, Int16
import topic_handler


DEFAULT_NAMESPACE = "tb3_"


def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)


wp.setup_node()
topic_handler.SubscribingHandler("robot_names", Int16MultiArray, callback)
rospy.spin()

namespace = rospy.get_param('~namespace', DEFAULT_NAMESPACE)
wp_manager = wp.WayPointManager(namespace=namespace, robot_names=["0", "1", "2", "3"])  # @HACK
wp_manager.run()
