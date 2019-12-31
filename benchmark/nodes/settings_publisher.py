#!/usr/bin/env python

""" --------------------------------------------------------------
@author:    Johann Schmidt
@date:      2019
@brief:     This will load the settings from the settings file to
            the parameter server.
@todo:
------------------------------------------------------------- """


import src.settings as settings
import rospy
import src.utils.naming_scheme as names


rospy.init_node(names.NodeNames.SETTINGS_PUBLISHER.value, anonymous=True)
manager = settings.SettingsManager()
manager.upload()
rospy.loginfo("Settings uploaded to parameter server!")
