#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Point

from benchmark.src import movement
from benchmark.src.utils import topic_handler
from benchmark.src.utils import naming_scheme as names

# import benchmark.src.movement as movement
# import benchmark.src.utils.topic_handler as topic_handler
# import benchmark.src.utils.naming_scheme as names


def callback_start_pos(data, args):
    """ Callback for robot start positions.
    :param data:
    :param args:
    """
    global start_pos, pos_received_flag
    start_pos[args[0]] = [data.x, data.y, data.z]
    pos_received_flag[args[0]] = True


def callback_target(data, args):
    """ Callback for robot target points to move to.
    :param data:
    :param args:
    """
    global robot_targets, robot_finished
    if not robot_finished[args[0]]:
        robot_targets[args[0]] = [data.x, data.y, data.z]


def callback_finished(data, args):
    """ Callback for rounds completed.
    :param data:
    :param args:
    """
    global robot_finished
    if data == Bool(True):
        robot_finished[args[0]] = True


def setup_move_controller(_namespace, _number_of_robots):
    """ Init. all required movement controller.
    :param _namespace:
    :param _number_of_robots:
    """
    for robot_id in range(_number_of_robots):
        move_controller[robot_id] = movement.MovementController(
            robot_name=str(robot_id), namespace=_namespace, waiting_time=5)


def setup_subscribers(_namespace, _number_of_robots):
    """ Setup all subscribers.
    :param _namespace:
    :param _number_of_robots:
    """
    global robot_finished, pos_received_flag
    for robot_id in range(_number_of_robots):
        pos_received_flag[robot_id] = False
        robot_finished[robot_id] = False
        topic_name = _namespace + \
            str(robot_id) + "/" + names.TopicNames.WAYPOINT.value
        topic_handler.SubscribingHandler(
            topic_name, Point, callback_target, robot_id)
        topic_name = _namespace + \
            str(robot_id) + "/" + names.TopicNames.FINISHED.value
        topic_handler.SubscribingHandler(
            topic_name, Bool, callback_finished, robot_id)
        topic_name = _namespace + \
            str(robot_id) + "/" + names.TopicNames.START_POSITION.value
        topic_handler.SubscribingHandler(
            topic_name, Point, callback_start_pos, robot_id)
    wait_for_pos()


def wait_for_targets(_number_of_robots, quiet=False, frequency=1):
    """ Waits until target points have been received.
    :param _number_of_robots:
    :param quiet:
    :param frequency:
    :return True:   received
            False:  waiting
    """
    while len(robot_targets) != _number_of_robots:
        if not quiet:
            rospy.loginfo("Waiting for target points ...")
        rospy.Rate(frequency).sleep()


def _apply_end_procedure(_robot_id):
    """ Starts the end procedure.
    :param _robot_id:
    """
    if end_procedure == 'despawn':
        # is handled by spawning controller
        pass
    elif end_procedure == 'stay':
        # do nothing
        pass
    elif end_procedure == 'idle':
        # up to the user
        pos = [1.8, 0, 0]
        move_controller[_robot_id].move_to(pos, quiet=False)

        pass
    elif end_procedure == 'start':
        move_controller[_robot_id].move_to(
            start_pos[_robot_id], quiet=False)


def update_movement(_number_of_robots, frequency=0.5):
    """ Updates the movement to the target points.
    :param _number_of_robots:
    :param frequency:
    """
    global robot_finished
    while not rospy.is_shutdown():
        for robot_id in range(_number_of_robots):
            if robot_id in robot_targets:
                if not robot_finished[robot_id]:
                    move_controller[robot_id].move_to(
                        robot_targets[robot_id], quiet=False)
                else:
                    _apply_end_procedure(robot_id)
        rospy.Rate(frequency).sleep()


def wait_for_pos():
    """ Waits until a start pos is received.
    """
    global pos_received_flag
    while not all(value == 0 for value in pos_received_flag.values()):
        rospy.Rate(0.5).sleep()
    rospy.loginfo("All start positions received!")


### main ###
# variables
move_controller = {}
robot_targets = {}
robot_finished = {}
start_pos = {}
pos_received_flag = {}

# initialise node
rospy.init_node(names.NodeNames.MOVEMENT_CONTROLLER.value, anonymous=True)

# params
namespace = rospy.get_param('namespace')
number_of_robots = rospy.get_param('number_of_robots')
end_procedure = rospy.get_param('end_procedure')

# setups
setup_subscribers(namespace, number_of_robots)
setup_move_controller(namespace, number_of_robots)

# execution
wait_for_targets(number_of_robots)
update_movement(number_of_robots)
