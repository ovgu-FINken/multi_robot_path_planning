#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Controller between launch file and python files and nodes.
@todo:
------------------------------------------------------------- """


import rospy
import src.spawner as sp
import src.formation as form
from std_msgs.msg import Int16MultiArray
import src.utils.topic_handler as topic_handler


publ = topic_handler.PublishingHandler('robot_names', Int16MultiArray, queue_size=10)
spawner = sp.RobotSpawner(world="world")

model_name = rospy.get_param('model_name')
model_type = rospy.get_param('model_type')
number_of_robots = rospy.get_param('number_of_robots')
namespace = rospy.get_param('namespace')
position = rospy.get_param('position')
orientation = rospy.get_param('orientation')
formation = rospy.get_param('formation')

formationHandler = form.FormationHandler(
    number_of_robots=number_of_robots, center_point=position,
    formation=formation, distance=0.2)
positions, orientations = formationHandler.run()
for i in range(number_of_robots):
    position = positions[i]
    orientation = orientations[i]
    spawner.spawn(
        model_name=model_name,
        model_type=model_type, namespace=namespace,
        position=position, orientation=orientation,
        name=str(i + 1), update_if_exist=False,
        use_launch_file=True)
spawner.spawn_via_launch(number_of_robots, positions)
publ.publish([j for j in range(number_of_robots)], quiet=False)
rospy.spin()
