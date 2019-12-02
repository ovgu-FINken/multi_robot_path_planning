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
from nav_msgs.msg import Odometry
import topic_handler
from geometry_msgs.msg import Point


DEFAULT_NAMESPACE = "tb3_"


# HACK: add arg to waypoint launch: number of robots
# and based on this generate the names
robot_names = ["1", "2", "3", "4"]
robot_current_positions = {}
publisher = {}


def callback_names(data, args):
    rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
    global robot_names
    robot_names = [str(name) for name in data.data]


def callback_target(name, point):
    global publisher
    publisher[name].publish(point, quiet=True)


def callback_odometry(data, args):
    #rospy.loginfo("Odometry: {0} and {1}".format(data.pose.pose.position, args))
    global robot_current_positions
    robot_current_positions[args[0]] = data.pose.pose.position


rospy.init_node("waypoint_controller", anonymous=True)
namespace = rospy.get_param('namespace', DEFAULT_NAMESPACE)
topic_handler.SubscribingHandler("robot_names", Int16MultiArray, callback_names)

while len(robot_names) == 0:
    rospy.loginfo("waiting")
    rospy.Rate(1).sleep()

for robot_name in robot_names:
    topic_name = namespace + robot_name + '/' + "waypoint"
    pub = topic_handler.PublishingHandler(topic_name, Point, queue_size=10)
    publisher[robot_name] = pub

    odom_name = "robot" + robot_name + "/odom"
    topic_handler.SubscribingHandler(odom_name, Odometry, callback_odometry, robot_name)

wp_manager = wp.WayPointManager(namespace=namespace, robot_names=robot_names,
                                callback=callback_target, waypoints=wp.WayPointMap.EDGE_TB3_WORLD)
# HACK
for robot_name in robot_names:
    wp_manager.next(robot_name)
while not rospy.is_shutdown():
    wp_manager.update(robot_current_positions, frequency=0.2)
    rospy.Rate(0.2).sleep()
