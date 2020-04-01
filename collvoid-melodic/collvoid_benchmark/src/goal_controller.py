#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
import threading
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult


class GoalController:
    def __init__(self):
        # self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=0)
        ns = rospy.get_namespace()
        topic_name = ns + "/benchmark/waypoint"
        self.sub = rospy.Subscriber(topic_name, Point, self.catch_waypoint_cb)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo_once("Waiting for MoveBaseAction-Server...")
        self.client.wait_for_server()
        rospy.loginfo_once("MoveBaseAction-Server done.")

    def catch_waypoint_cb(self, data):
        '''
        @brief: enhance waypoint with orientation, frame_id and timestamp
        '''
        self.cntr = 0
        waypoint = data
        nextGoal = PoseStamped()
        nextGoal.header.stamp = rospy.Time.now()
        nextGoal.header.frame_id = "map"
        nextGoal.pose.position = waypoint
        nextGoal.pose.orientation.x = 0
        nextGoal.pose.orientation.y = 0
        nextGoal.pose.orientation.z = 0
        nextGoal.pose.orientation.w = 1

        self.forward_goal(nextGoal)

    def forward_goal(self, nextGoal):
        '''
        @brief: send goal using service API of move_base
        '''
        goal = MoveBaseActionGoal()
        goal = nextGoal
        self.client.send_goal()
        self.client.wait_for_result()
        # result = self.client.get_result()

        if self.cntr < 3 and self.client.get_state() == actionlib.GoalStatus.ABORTED:
            self.cntr += 1
            self.forward_goal(goal)
        elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.spin()
        else:
            rospy.logerr("Goal cannot be reached, tried 3 times...")


if __name__ == '__main__':
    rospy.init_node('goal_controller', anonymous=True)

    try:
        GoalController()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
