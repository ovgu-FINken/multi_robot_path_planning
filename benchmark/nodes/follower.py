#!/usr/bin/env python
from __future__ import division
import actionlib
import math
import random
import rospy
import tf2_ros
import scipy.stats
from geometry_msgs.msg import Quaternion, PointStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal


# https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
def euler_to_quaternion(yaw, pitch, roll):  # yaw (Z), pitch (Y), roll (X)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = Quaternion()

    q.w = cy * cp * cr + sy * sp * sr
    q.x = cy * cp * sr - sy * sp * cr
    q.y = sy * cp * sr + cy * sp * cr
    q.z = sy * cp * cr - cy * sp * sr
    return q


class Follower:
    def walk(self):
        client = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        client.wait_for_server()

        goal = MoveBaseGoal()

        goal.target_pose.header.frame_id = "tb3_1/base_footprint"
        goal.target_pose.header.stamp = rospy.Time.now()-rospy.Duration(0.1)

        goal.target_pose.pose.position.x = 0
        goal.target_pose.pose.position.y = 0

        angle = random.uniform(-math.pi, math.pi)
        rotation = euler_to_quaternion(angle, 0, 0)
        goal.target_pose.pose.orientation = rotation
        rospy.loginfo(goal)
        if goal is None:
            return
        client.send_goal_and_wait(
            goal, execute_timeout=rospy.Duration.from_sec(execute_timeout))

if __name__ == '__main__':

    rospy.init_node('follower', anonymous=True)

    namespace = rospy.get_namespace()

    execute_timeout = rospy.get_param('~execute_timeout', 5)

    follower = Follower()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        follower.walk()
        rate.sleep()
