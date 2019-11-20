#!/usr/bin/env python
import rospy

# Because of transformations
import tf_conversions
import tf2_ros
import geometry_msgs.msg


def handle_turtle_pose(botname):
#    br = tf2_ros.TransformBroadcaster()
# using static transformation because it remains the same for the whole simulation session and thus, only needs to be braodcasted once

    br = tf2_ros.StaticTransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = botname + "/map"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.0
    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('global_map_broadcaster')
    botname = rospy.get_param('~turtlebot')
    handle_turtle_pose(botname)
    rospy.spin()

#    rate = rospy.Rate(10)
#    while not rospy.is_shutdown():
#    	handle_turtle_pose(botname)
#    	rate.sleep()
