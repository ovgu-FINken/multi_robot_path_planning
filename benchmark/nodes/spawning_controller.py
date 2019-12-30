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
import src.utils.topic_handler as topic_handler
from std_msgs.msg import Bool
from geometry_msgs.msg import Point


def callback_rounds(data, args):
    """ Callback when the rounds are fulfilled.
    :param data:
    :param args:
    """
    global spawner, namespace
    if data == Bool(True) and end_procedure == 'despawn':
        spawner.despawn_robot(str(namespace) + str(args[0]))


def run_formation(_number_of_robots, _position,
                  _orientation, _formation, distance=0.2):
    """ Starts the formation handler.
    :param _number_of_robots:
    :param _position:
    :param _orientation:
    :param distance:
    :param _formation:
    :return positions, orientations
    """
    handler = form.FormationHandler(
        number_of_robots=_number_of_robots, center_point=_position,
        formation=_formation, distance=distance)
    _positions, _orientations = handler.run()
    return _positions, _orientations


def spawn_robots(_positions, _orientations, _number_of_robots,
                 _model_name, _model_type, _namespace):
    """ Spawns the robots via the spawning controller.
    :param _number_of_robots:
    :param _positions:
    :param _orientations:
    :param _model_name:
    :param _model_type:
    :param _namespace:
    """
    for i in range(_number_of_robots):
        _position = _positions[i]
        _orientation = _orientations[i]
        spawner.spawn(
            model_name=_model_name,
            model_type=_model_type, namespace=_namespace,
            position=_position, orientation=_orientation,
            name=str(i), update_if_exist=False,
            use_launch_file=True)
    spawner.spawn_via_launch(_number_of_robots, _positions)


def setup_subscriber(_number_of_robots, _namespace):
    """ Setup for subscriber.
    :param _number_of_robots:
    :param _namespace:
    """
    for robot_id in range(number_of_robots):
        topic_name = namespace + str(robot_id) + "/rounds"
        topic_handler.SubscribingHandler(topic_name, Bool, callback_rounds, robot_id)


def publish_start_position(_namespace, _positions, frequency=3):
    """ Publishes the start position of the robot.
    :param _namespace:
    :param _positions:
    :param frequency:
    """
    for robot_id in range(number_of_robots):
        name = _namespace + str(robot_id) + "/start_pos"
        pub = topic_handler.PublishingHandler(name, Point)
        pub.publish(_positions[robot_id])


spawner = sp.RobotSpawner(world="world")
model_name = rospy.get_param('model_name')
model_type = rospy.get_param('model_type')
number_of_robots = rospy.get_param('number_of_robots')
namespace = rospy.get_param('namespace')
position = rospy.get_param('position')
orientation = rospy.get_param('orientation')
formation = rospy.get_param('formation')
end_procedure = rospy.get_param('end_procedure')
setup_subscriber(number_of_robots, namespace)
positions, orientations = run_formation(
    number_of_robots, position, orientation, formation)
publish_start_position(namespace, positions)
spawn_robots(
    positions, orientations, number_of_robots,
    model_name, model_type, namespace)
