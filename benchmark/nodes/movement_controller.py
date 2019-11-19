#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Node for movement
@todo:
------------------------------------------------------------- """


import rospy
import movement
import topic_handler
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Point


robot_names = []
robot_targets = {}


def callback_names(data):
    global robot_names
    robot_names = [str(name) for name in data.data]
    rospy.loginfo("Names: {}".format(robot_names))


def callback_target(data, *args):
    global robot_names
    rospy.loginfo("Data: {}".format(data.data))
    rospy.loginfo("Args: {}".format(args))
    robot_targets[args[0]] = data.data


rospy.init_node('movement_controller', anonymous=True)
topic_handler.SubscribingHandler("robot_names", Int16MultiArray, callback_names)
namespace = "tb3"
move_controller = {}
for robot_name in robot_names:
    topic_name = namespace + '_' + robot_name + '/' + "waypoint"
    topic_handler.SubscribingHandler(topic_name, Point, callback_target, robot_name)
    move_controller[robot_name] = movement.MovementController(namespace + '_' + robot_name)

while not rospy.is_shutdown():
    for robot_name in robot_names:
        try:
            move_controller[robot_name].linear_move_to(robot_targets[robot_name])
        except KeyError:
            pass
    rospy.Rate(1).sleep()
