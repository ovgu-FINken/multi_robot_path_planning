#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Script used to spawn a turtlebot in a generic position
@todo:
------------------------------------------------------------- """

import sys
import roslib
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest, SpawnModelRequest
import rospy
import os
from geometry_msgs.msg import *
import tf.transformations as tft
from std_msgs.msg import Empty as EmptyMsg
import formation as form
from geometry_msgs.msg import Quaternion
import math


NODE_NAME = "spawn_robot"
DEFAULT_MODEL_NAME = "turtlebot3"
DEFAULT_MODEL_TYPE = "burger"
DEFAULT_NUMBER_OF_ROBOTS = 3
DEFAULT_POSITION = [0, 0, 0]
DEFAULT_ORIENTATION = [0, 0, 0]
DEFAULT_NAME = "0"
DEFAULT_NAMESPACE = "tb3_"
DEFAULT_FORMATION = form.Formation.DENSE_BLOCK


def get_obj_pose(position, orientation):
    """ Returns the pose of the object.
    :param position:
    :param orientation:
    :return: pose
    """
    quaternion = tft.quaternion_from_euler(orientation[0], orientation[1], orientation[2])
    object_pose = Pose()
    object_pose.position.x = float(position[0])
    object_pose.position.y = float(position[1])
    object_pose.position.z = float(position[2])
    object_pose.orientation.x = quaternion[0]
    object_pose.orientation.y = quaternion[1]
    object_pose.orientation.z = quaternion[2]
    object_pose.orientation.w = quaternion[3]
    return object_pose


def get_file_location(model, model_type):
    """ Returns the file location path.
    :param model:
    :param model_type:
    :return: path
    """
    try:
        file_location = roslib.packages.get_pkg_dir(
            'turtlebot3_gazebo') + '/models/' + model + '_' + model_type
    except:
        rospy.logerr("File not found: turtlebot3_gazebo" + "/models/" + model + "_" + model_type)
        return None
    return file_location


def load_model(file_path, model_format="sdf"):
    """ Call gazebo service to spawn model (see http://ros.org/wiki/gazebo).
    :param file_path:
    :param model_format:
    :return: srv_model, xml_string
    """
    srv_spawn_model, xml_string = None, None
    if model_format == "urdf":
        srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        file_xml = open(file_path)
        xml_string = file_xml.read()
    elif model_format == "urdf.xacro":
        p = os.popen("rosrun xacro xacro.py " + file_path)
        xml_string = p.read()
        p.close()
        srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
    elif model_format == "sdf":
        srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_gazebo_model', SpawnModel)
        file_xml = open(os.path.join(file_path, "model.sdf"))
        xml_string = file_xml.read()
    else:
        rospy.logerr('Model type not know. model_type = ' + model_format)
    return srv_spawn_model, xml_string


def delete_robot(name):
    """ Deletes a robot.
    :param name:
    :return: request
    """
    srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
    req = DeleteModelRequest()
    req.model_name = name
    exists = True
    try:
        res = srv_delete_model(name)
    except rospy.ServiceException as e:
        exists = False
        rospy.logdebug("Model %s does not exist in gazebo.", name)
    if exists:
        rospy.loginfo("Model %s already exists in gazebo. Model will be updated.", name)
    return req


def spawn_model(xml_string, name, namespace, model, pose, world):
    """ Spawns a model in a simulation world.
    :param name:
    :param namespace:
    :param model:
    :param pose:
    :param world:
    """
    rospy.wait_for_service('gazebo/spawn_sdf_model')
    spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
    spawn_model_prox(name, xml_string, namespace, pose, world)


def spawn_robot(name, model_name, namespace, model_type,
                position, orientation, update_if_exist=False):
    """ Spawns a robot.
    :param name:
    :param model_name:
    :param namespace:
    :param model_type:
    :param position:
    :param orientation:
    :param update_if_exist:
    """
    pose = get_obj_pose(position, orientation)
    file_path = get_file_location(model_name, model_type)
    srv_spawn_model, xml_string = load_model(file_path)

    if update_if_exist:
        delete_robot(name)

    spawn_model(xml_string, name, namespace, srv_spawn_model, pose, world="world")


def create_node(name=NODE_NAME):
    """ Creates a node.
    :param name:
    """
    rospy.init_node(name, anonymous=True)
    rospy.sleep(5)
    rospy.wait_for_service("/gazebo/spawn_urdf_model")


def loop():
    """ Loop until shutdown.
    """
    sim = rospy.get_param('/use_sim_time')
    if sim is True:
        rospy.loginfo('Running in simulation, publishing to /sim_spawned topic')
        pub = rospy.Publisher('/sim_spawned', EmptyMsg, latch=True)
        pub.publish(EmptyMsg())
        pub.publish(EmptyMsg())
        pub.publish(EmptyMsg())
        rospy.spin()
