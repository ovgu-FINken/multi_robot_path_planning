#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Point, PoseStamped
import threading
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseResult, MoveBaseFeedback


class GoalController:
    def __init__(self):
        ns = rospy.get_namespace()

        # init publisher for simple goal
        self.pub = rospy.Publisher(
            'move_base_simple/goal', PoseStamped, queue_size=0)

        # init subscriber to waypoints
        topic_wp = ns + "benchmark/waypoint"
        self.sub = rospy.Subscriber(topic_wp, Point, self.catch_waypoint_cb)

        # init move base action client
        # topic_srv = ns + "move_base"
        topic_srv = "move_base"
        client = actionlib.SimpleActionClient(topic_srv, MoveBaseAction)
        rospy.loginfo_once("Waiting for MoveBaseAction-Server...")
        rospy.loginfo("Client for topic " + topic_srv)
        if (client.wait_for_server()):
            rospy.loginfo_once("MoveBaseAction client setup finished.")
            self.use_srv = True
            self._client = client

    def catch_waypoint_cb(self, data):
        '''
        @brief: enhance waypoint with orientation, frame_id and timestamp
        '''
        rospy.loginfo("Received goal: %f, %f, %f", data.x, data.y, data.z)
        self.cntr = 0

        if (self.use_srv):
            self.forward_goal_as_service(data)
        else:
            self.forward_goal_as_message(data)

    def forward_goal_as_message(self, nextWp):
        '''
        @brief: send goal using topic API of move_base
        '''
        nextPose = PoseStamped()
        nextPose.header.stamp = rospy.Time.now()
        nextPose.header.frame_id = "map"
        nextPose.pose.position = nextWp
        nextPose.pose.orientation.x = 0
        nextPose.pose.orientation.y = 0
        nextPose.pose.orientation.z = 0
        nextPose.pose.orientation.w = 1
        self.pub.publish(nextPose)
        rospy.loginfo("Forwarded goal: %f, %f, %f", nextPose.pose.position.x,
                      nextPose.pose.position.y, nextPose.pose.position.z)

    def forward_goal_as_service(self, nextWp):
        '''
        @brief: send goal using service API of move_base
        '''
        nextGoal = MoveBaseGoal()
        nextGoal.target_pose.header.stamp = rospy.Time.now()
        nextGoal.target_pose.header.frame_id = "map"
        nextGoal.target_pose.pose.position = nextWp
        # nextGoal.target_pose.pose.position.x = data.x
        # nextGoal.target_pose.pose.position.y = data.y
        # nextGoal.target_pose.pose.position.z = data.z
        nextGoal.target_pose.pose.orientation.x = 0
        nextGoal.target_pose.pose.orientation.y = 0
        nextGoal.target_pose.pose.orientation.z = 0
        nextGoal.target_pose.pose.orientation.w = 1

        self._client.send_goal(nextGoal)
        rospy.loginfo("Forwarded goal: %f, %f, %f", nextGoal.target_pose.pose.position.x,
                      nextGoal.target_pose.pose.position.y, nextGoal.target_pose.pose.position.z)

        self._client.wait_for_result()
        result = self._client.get_result()
        rospy.loginfo("Result: %s", result)

        if self.cntr < 3 and self._client.get_state() == actionlib.GoalStatus.ABORTED:
            self.cntr += 1
            rospy.logerr(
                "Goal could not be reached, try again [%f/3].", self.cntr)
            self.forward_goal(nextGoal)
        elif self._client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.spin()
        else:
            rospy.logerr("Goal cannot be reached, tried 3 times...")


if __name__ == '__main__':
    rospy.init_node('goal_controller', anonymous=True)
    rospy.loginfo("Initialising goal controller...")

    try:
        GoalController()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()
