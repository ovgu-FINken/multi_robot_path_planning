#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Controller between launch file and python files and nodes.
@todo:
------------------------------------------------------------- """


import rospy
import waypoint as wp
from std_msgs.msg import Int16MultiArray
import topic_handler
from geometry_msgs.msg import Point


DEFAULT_NAMESPACE = "tb3_"


robot_names = []
publisher = {}


def callback_names(data, args):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global robot_names
    robot_names = [str(name) for name in data.data]


def callback_target(name, point):
    global publisher
    publisher[name].publish(point, quiet=False)


rospy.init_node("waypoint_controller", anonymous=True)
namespace = rospy.get_param('~namespace', DEFAULT_NAMESPACE)
topic_handler.SubscribingHandler("robot_names", Int16MultiArray, callback_names)

while len(robot_names) == 0:
    rospy.loginfo("waiting")
    rospy.Rate(1).sleep()

for robot_name in robot_names:
    topic_name = namespace + robot_name + '/' + "waypoint"
    pub = topic_handler.PublishingHandler(topic_name, Point, queue_size=10)
    publisher[robot_name] = pub

wp_manager = wp.WayPointManager(namespace=namespace, robot_names=robot_names,
                                callback=callback_target, waypoints=wp.WayPointMap.EDGE_TB3_WORLD)
# HACK
wp = wp_manager.next(robot_names[0])
while not rospy.is_shutdown():
    rospy.Rate(1).sleep()
