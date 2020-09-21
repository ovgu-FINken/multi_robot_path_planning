#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import PoseWithCovarianceStamped
from gazebo_msgs.msg import ModelStates

class InitialPosePublisher:
    def __init__(self):
        self.poses = {} 
        self._ns = rospy.get_namespace()
        self.done = False
        self.publisher = rospy.Publisher("%sinitialpose"%self._ns, PoseWithCovarianceStamped, queue_size=0, latch=True)
        self.pose_sub = rospy.Subscriber("/gazebo/model_states", ModelStates, self._model_states_cb)
        self.counter = 0

    def _model_states_cb(self, msg):
        if self.done:
            return
        rospy.loginfo("got pose estimates")
        for i, key in enumerate(msg.name):
            if key == self._ns[1:-1]:
                rospy.loginfo("found robot %s"%key)
                self.publish(msg.pose[i])
                self.done = True

    def publish(self, pose):
        pwcs = PoseWithCovarianceStamped()
        pwcs.pose.pose = pose
        pwcs.pose.covariance[0] = 0.5 ** 2
        pwcs.pose.covariance[7] = 0.5 ** 2
        pwcs.pose.covariance[35] = math.pi/12.0 * math.pi/12.0

        pwcs.header.stamp = rospy.Time.now()
        pwcs.header.frame_id = "map"
        self.publisher.publish(pwcs)


if __name__ == '__main__':
    rospy.init_node('initial_pose_publisher')
    ipp = InitialPosePublisher()
    rospy.spin()


