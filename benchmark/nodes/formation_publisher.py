#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Publishes the formation poses for each robot.
@todo:
------------------------------------------------------------- """


import rospy
import spawner as sp
import waypoint as wp
import formation as form
from std_msgs.msg import Int16MultiArray, MultiArrayLayout, MultiArrayDimension
from std_msgs.msg import Empty as EmptyMsg
import topic_handler


DEFAULT_NUMBER_OF_ROBOTS = 3
DEFAULT_NAMESPACE = "tb3_"
DEFAULT_FORMATION = form.Formation.DENSE_BLOCK


publ = topic_handler.PublishingHandler('formation', Int16MultiArray, queue_size=10)


number_of_robots = rospy.get_param('~number_of_robots', DEFAULT_NUMBER_OF_ROBOTS)
namespace = rospy.get_param('~namespace', DEFAULT_NAMESPACE)

formationHandler = form.FormationHandler(
    number_of_robots=number_of_robots, center_point=[1.5, 0.5, 0.5],
    formation=DEFAULT_FORMATION, distance=0.2)
positions, orientations = formationHandler.run()
publ.publish(positions, quiet=False)
rospy.spin()
