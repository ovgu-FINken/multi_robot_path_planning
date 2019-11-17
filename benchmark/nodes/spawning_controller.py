#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Controller between launch file and python files and nodes.
@todo:
------------------------------------------------------------- """


import rospy
import spawner as sp
import waypoint as wp
import formation as form


DEFAULT_MODEL_NAME = "turtlebot3"
DEFAULT_MODEL_TYPE = "burger"
DEFAULT_NUMBER_OF_ROBOTS = 3
DEFAULT_POSITION = [0, 0, 0]
DEFAULT_ORIENTATION = [0, 0, 0]
DEFAULT_NAME = "0"
DEFAULT_NAMESPACE = "tb3_"
DEFAULT_FORMATION = form.Formation.DENSE_BLOCK


spawner = sp.RobotSpawner(world="world")

model_name = rospy.get_param('~model_name', DEFAULT_MODEL_NAME)
model_type = rospy.get_param('~model_type', DEFAULT_MODEL_TYPE)
number_of_robots = rospy.get_param('~number_of_robots', DEFAULT_NUMBER_OF_ROBOTS)
namespace = rospy.get_param('~namespace', DEFAULT_NAMESPACE)
position = rospy.get_param('~position', [1.5, 0.5, 0.5])#DEFAULT_POSITION)
orientation = rospy.get_param('~orientation', DEFAULT_ORIENTATION)
formation = rospy.get_param('~formation', DEFAULT_FORMATION)

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
        name=str(i), update_if_exist=False)
spawner.update()
