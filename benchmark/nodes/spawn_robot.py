#!../venv/bin/python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Script used to spawn a turtlebot in a generic position
@todo:
------------------------------------------------------------- """

import rospy
import os
import sys
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import SpawnEntity


NODE_NAME = "entity_spawner"
DEFAULT_MODEL_TYPE = "burger"
DEFAULT_NUMBER_OF_ROBOTS = 3
DEFAULT_POSITION = [0, 0, 0]
DEFAULT_NAME = "0"
DEFAULT_NAMESPACE = "tb3_"


def node_logger(node, text):
    """ Logs a text for a node.
    :param node:
    :param text:
    """
    if node is not None:
        node.get_logger().info(text)


def create_node(name):
    """ Creates a node.
    :param name
    :return node
    """
    rclpy.init()
    node = rclpy.create_node(name)
    node_logger(node, 'Creating Service client to connect to `/' + name + 'spawn_entity`')
    return node


def create_client(node, name):
    """ Creates a client.
    :param node:
    :param name:
    :return: client
    """
    client = node.create_client(SpawnEntity, "/" + name)
    node_logger(node, "Connecting to `/" + name + "` service...")
    if not client.service_is_ready():
        client.wait_for_service()
        node_logger(node, "...connected!")
    return client


def spawn_robot(_model_type, _name_space, _name, _position):
    """ Spawns a turtlebot.
    :param _model_type:
    :param _name_space:
    :param _name:
    :param _position:
    """
    node = create_node(NODE_NAME)
    client = create_client(node, NODE_NAME)

    sdf_file_path = os.path.join(
        get_package_share_directory("turtlebot3_gazebo"), "models",
        "turtlebot3_", model_type, "model.sdf")

    request = SpawnEntity.Request()
    request.name = _name
    request.xml = open(sdf_file_path, 'r').read()
    request.robot_namespace = _name_space
    request.initial_pose.position.x = float(_position[0])
    request.initial_pose.position.y = float(_position[1])
    request.initial_pose.position.z = float(_position[2])

    node.get_logger().info("Sending service request to `/spawn_entity`")
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    if future.result() is not None:
        print('response: %r' % future.result())
    else:
        raise RuntimeError(
            'exception while calling service: %r' % future.exception())

    node.get_logger().info("Done! Shutting down node.")
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    """ Main method. """
    model_type = rospy.get_param('~model_type', DEFAULT_MODEL_TYPE)
    number_of_robots = rospy.get_param('~number_of_robots', DEFAULT_NUMBER_OF_ROBOTS)
    namespace = rospy.get_param('~namespace', DEFAULT_NAMESPACE)
    position = rospy.get_param('~position', DEFAULT_POSITION)

    for i in range(number_of_robots):
        spawn_robot(
            _model_type=model_type, _name_space=namespace,
            _position=position, _name=str(i))

