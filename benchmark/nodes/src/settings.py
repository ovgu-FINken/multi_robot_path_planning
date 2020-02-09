""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019
@brief:     JSON settings file manager.
@todo:
------------------------------------------------------------- """


import json
import os
import src.formation as form
import rospy
import rospkg


DEFAULT_MODEL_NAME = "turtlebot3"
DEFAULT_MODEL_TYPE = "burger"
DEFAULT_NUMBER_OF_ROBOTS = 4
DEFAULT_POSITION = [0, 0, 0]
DEFAULT_ORIENTATION = [0, 0, 0]
DEFAULT_NAME = "0"
DEFAULT_NAMESPACE = "tb3_"
DEFAULT_FORMATION = form.Formation.DENSE_BLOCK

DEFAULT_SETTINGS_PATH = rospkg.RosPack().get_path('benchmark') + "/settings/settings.json"


class SettingsManager:
    """ Settings Manager.
    """

    def __init__(self, path=DEFAULT_SETTINGS_PATH):
        """ Init. method.
        :param path:
        """
        self._path = path
        self._file = open(path)
        self._data = json.load(self._file)

    def read(self, item):
        """ Returns the value of an item.
        :param item
        """
        return self._data[item]

    def upload(self):
        """ This uploads all settings parameters to the parameter server.
        """
        for item in self._data.keys():
            rospy.set_param(item, self._data[item])
