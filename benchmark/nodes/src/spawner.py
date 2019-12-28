#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      November 2019
@brief:     Script used to spawn a turtlebot in a generic position
@todo:
------------------------------------------------------------- """

import roslib
from gazebo_msgs.srv import SpawnModel, DeleteModel, DeleteModelRequest
import rospy
import os
from std_msgs.msg import Empty as EmptyMsg
import src.utils.ros_utils as ros_utils
import rospkg
import yaml


NODE_NAME = "spawning_controller"


class RobotSpawner:
    """ Robot spawner.
    """

    def __init__(self, world):
        """ Init. method.
        :param world
        """
        self._world = world
        self._create_node()
        self._name = None
        self._model_type = None
        self._model_format = None
        self._namespace = None
        self._position = None
        self._orientation = None
        self._param_file = None
        self._active = {}
        self._number_of_robots = 0

    @staticmethod
    def _get_file_location(model_name, model_type, model_format="sdf"):
        """ Returns the file location path.
        :param model_name:
        :param model_type:
        :param model_format:
        :return: path
        """
        package, package_dir, file_name = "", "", ""
        if model_format == "sdf":
            package = 'turtlebot3_gazebo'
            file_name = '/models/' + model_name + '_' + model_type + "/model.sdf"
        elif model_format == "urdf.xacro":
            package = 'turtlebot3_description'
            file_name = '/urdf/' + model_name + "_" + model_type + ".urdf.xacro"
        else:
            rospy.logerr("Model format " + model_format + " not supported!")
        try:
            package_dir = roslib.packages.get_pkg_dir(package)
        except:
            rospy.logerr("Package ({}) not found!".format(package))
        return package_dir + file_name

    @staticmethod
    def _load_model(file_path, model_format="sdf"):
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
            p = os.popen("rosrun xacro --inorder " + file_path)
            xml_string = p.read()
            p.close()
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_urdf_model', SpawnModel)
        elif model_format == "sdf":
            srv_spawn_model = rospy.ServiceProxy('/gazebo/spawn_gazebo_model', SpawnModel)
            file_xml = open(file_path)
            xml_string = file_xml.read()
        else:
            rospy.logerr('Model type not know. model_type = ' + model_format)
        return srv_spawn_model, xml_string

    def despawn_robot(self, model_name, quiet=False):
        """ Deletes the robot.
        :param model_name
        :param quiet:
        """
        if not self._active[model_name]:
            if not quiet:
                rospy.loginfo("Robot {} not active".format(model_name))
            return
        rospy.loginfo("Despawning robot {}".format(model_name))
        self._activate(False, model_name)
        srv_delete_model = rospy.ServiceProxy('gazebo/delete_model', DeleteModel)
        try:
            srv_delete_model(str(model_name))
        except rospy.ServiceException:
            if not quiet:
                rospy.logdebug("Model %s does not exist in gazebo.", model_name)

    def _spawn_model(self, xml_string, name, namespace, pose):
        """ Spawns a model in a simulation world.
        :param name:
        :param namespace:
        :param model:
        :param pose:
        :param world:
        """
        rospy.wait_for_service('gazebo/spawn_sdf_model')
        spawn_model_prox = rospy.ServiceProxy('gazebo/spawn_sdf_model', SpawnModel)
        spawn_model_prox(name, xml_string, namespace, pose, self._world)

    def _setup_param_file(self):
        """ Creates a param file for the specific robot.
        :return: param file name
        """
        data = dict(
            model=self._model_type,
            name=self._name,
            namespace=self._namespace,
            pose_x=str(self._position[0]),
            pose_y=str(self._position[1]),
            pose_z=str(self._position[2]),
            simulate="True",
            auto_drive="False")

        rospack = rospkg.RosPack()
        directory = rospack.get_path('benchmark')
        # HACK
        directory = "/home/johschm/git/DrivingSwarm/src/pathplanning/benchmark/config"
        file_name = os.path.join(directory, "robot_" + self._name + "_params.yaml")
        file = open(file_name, "w")
        yaml.dump(data, file, default_flow_style=False)
        file.close()
        self._param_file = file_name
        return file_name

    def spawn_via_launch(self, number, positions):
        """ Spawns a robot using a spawning launch file.
        """
        self._number_of_robots = number
        self._activate(True)
        rospy.loginfo(self._active)
        str_pos = ["'", "'"]
        for position in positions:
            str_pos[0] += str(position[0]) + " "
            str_pos[1] += str(position[1]) + " "
        str_pos[0] = str_pos[0][:-1] + "'"
        str_pos[1] = str_pos[1][:-1] + "'"
        os.system("roslaunch benchmark spawn.launch"
                  + " namespace:=" + self._namespace
                  + " nr:=" + str(number - 1)
                  + " pose_x:=" + str_pos[0]
                  + " pose_y:=" + str_pos[1])

    def _activate(self, activate, robot_name=None):
        """ Sets the activate flag. This flag is utilized to verify if a certain robot is
        active in the world (spawned) or not (despawned) to avoid redundant calls.
        :param robot_name: if None: all robots are set to be active.
        :param activate: activate flag
        """
        if robot_name is None:
            if self._number_of_robots is None or self._number_of_robots <= 0:
                return
            for robot_name in range(self._number_of_robots):
                self._active[self._namespace + str(robot_name)] = activate
        else:
            self._active[robot_name] = activate

    def spawn(self, name, model_name, namespace, model_type,
              position, orientation, update_if_exist=False,
              use_launch_file=False, model_format="sdf"):
        """ Spawns a robot.
        :param name:
        :param model_name:
        :param namespace:
        :param model_type:
        :param position:
        :param orientation:
        :param update_if_exist:
        :param use_launch_file
        :param model_format:
        """
        self._name = name
        self._model_type = model_type
        self._model_format = model_format
        self._namespace = namespace
        self._position = position
        self._orientation = orientation

        if use_launch_file:
            self._setup_param_file()
            #self._spawn_via_launch()
        else:
            pose = ros_utils.get_obj_pose(position, orientation)
            file_path = self._get_file_location(model_name, model_type, model_format)
            srv_spawn_model, xml_string = self._load_model(file_path, model_format)

            if update_if_exist:
                self._delete_robot(name)

            self._spawn_model(xml_string=xml_string, name=name, namespace=namespace, pose=pose)
            self._pub_sim_spawned()

    @staticmethod
    def _pub_sim_spawned():
        """ Publishes an empty msg to sim_spawned if sim_time is used.
        """
        sim = rospy.get_param('/use_sim_time')
        if sim is True:
            rospy.loginfo('Running in simulation, publishing to /sim_spawned topic')
            pub = rospy.Publisher('/sim_spawned', EmptyMsg,
                                  latch=True, queue_size=10)
            pub.publish(EmptyMsg())
            pub.publish(EmptyMsg())
            pub.publish(EmptyMsg())

    @staticmethod
    def _create_node():
        """ Creates a node.
        """
        rospy.init_node(NODE_NAME, anonymous=True)
        rospy.sleep(5)
        rospy.wait_for_service("/gazebo/spawn_urdf_model")

