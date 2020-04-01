#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
import threading
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionGoal, MoveBaseActionFeedback, MoveBaseActionResult


class GoalController:
    def __init__(self):
        self.pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=0)
        ns = rospy.get_namespace()
        topic_name = ns + "benchmark/waypoint"
        self.sub = rospy.Subscriber(topic_name, Point, self.catch_waypoint_cb)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo_once("Waiting for MoveBaseAction-Server...")
        self.client.wait_for_server()
        rospy.loginfo_once("MoveBaseAction-Server done.")

    def catch_waypoint_cb(self, data):
        '''
        @brief: enhance waypoint with orientation, frame_id and timestamp
        '''
        rospy.loginfo("Received goal: %f, %f, %f", data.x, data.y, data.z)
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

        # self.forward_goal_as_service(nextGoal)
        self.forward_goal_as_message(nextGoal)

    def forward_goal_as_message(self, nextGoal):
        goal = PoseStamped()
        goal = nextGoal
        self.pub.publish(goal)
        rospy.loginfo("Forwarded goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)

    def forward_goal_as_service(self, nextGoal):
        '''
        @brief: send goal using service API of move_base
        '''

        goal = MoveBaseActionGoal()
        goal = nextGoal
        self.client.send_goal(goal)
        rospy.loginfo("Forwarded goal: %f, %f, %f", goal.pose.position.x, goal.pose.position.y, goal.pose.position.z)

        self.client.wait_for_result()
        result = self.client.get_result()
        rospy.loginfo("Result: %s", result)

        if self.cntr < 3 and self.client.get_state() == actionlib.GoalStatus.ABORTED:
            rospy.logerr("Goal could not be reached, try again...")
            self.cntr += 1
            self.forward_goal(goal)
        elif self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.spin()
        else:
            rospy.logerr("Goal cannot be reached, tried 3 times...")


if __name__ == '__main__':
    rospy.init_node('goal_controller', anonymous=True)
    rospy.loginfo("Goal controller initiated.")

    try:
        GoalController()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
